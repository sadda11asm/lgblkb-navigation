import geojson
import numpy as np
from box import Box
from lgblkb_tools import logger
from matplotlib import pyplot as plt
from matplotlib.path import Path
from numpy import ones,concatenate,asarray
from shapely import geometry as shg

def ring_coding(ob):
	# The codes will be all "LINETO" commands, except for "MOVETO"s at the
	# beginning of each subpath
	n=len(ob.coords)
	codes=ones(n,dtype=Path.code_type)*Path.LINETO
	codes[0]=Path.MOVETO
	return codes

def pathify(polygon):
	# Convert coordinates to path vertices. Objects produced by Shapely's
	# analytic methods have the proper coordinate order, no need to sort.
	vertices=concatenate(
		[asarray(polygon.exterior)]
		+[asarray(r) for r in polygon.interiors])
	codes=concatenate(
		[ring_coding(polygon.exterior)]
		+[ring_coding(r) for r in polygon.interiors])
	return Path(vertices,codes)

def perpendicular(a):
	b=np.empty_like(a)
	b[0]=-a[1]
	b[1]=a[0]
	return b

def normalize(a):
	a=np.array(a)
	return a/np.linalg.norm(a)

def plot_line(line):
	return plt.plot(*line.xy,lw=5)

round_decimals=6
min_dist=1e-6

class GenericGeometry(object):
	
	def __init__(self,**data):
		self.data=Box(data,ordered_box=True)
	
	# self.id=uuid.uuid4()
	
	# def _generate_id(self,id_obj=None):
	# 	self.id=self.data.pop('id',gsup.get_md5(str(id_obj or self.geometry)))
	
	def __repr__(self):
		return f"{self.__class__.__name__}: {self.geometry}"
	
	def __getitem__(self,item):
		# logger.debug('item: %s',item)
		return self.data[item]
		# raise KeyError('Invalid key provided.',dict(key=item,key_type=type(item)))
		pass
	
	@property
	def geometry(self):
		raise NotImplementedError
	
	def almost_touches(self,other):
		return self.geometry.distance(other.geometry)<min_dist

def line_xy(linestring):
	return np.array(linestring.xy).T

def get_rounded(linestring,decimals):
	new_linestring=shg.LineString(np.round(line_xy(linestring),decimals=decimals))
	return new_linestring

def get_lines(*polygons):
	lines=list()
	for pol in [polygons]:
		boundary=pol.boundary
		if boundary.type=='MultiLineString':
			for line in boundary:
				lines.append(line)
		else:
			lines.append(boundary)
	return lines

def coors_to_img(coors):
	# get ranges
	xmin=coors[:,0].min()
	xmax=coors[:,0].max()
	ymin=coors[:,1].min()
	ymax=coors[:,1].max()
	zmin=coors[:,2].min()
	zmax=coors[:,2].max()
	
	# create array for image : zmax+1 is the default value
	shape=(xmax-xmin+1,ymax-ymin+1)
	img=np.ma.array(np.ones(shape)*(zmax+1))
	
	for inp in coors:
		img[inp[0]-xmin,inp[1]-ymin]=inp[2]
	
	# set mask on default value
	img.mask=(img==zmax+1)
	
	# set a gray background for test
	img_bg_test=np.zeros(shape)
	cmap_bg_test=plt.get_cmap('gray')
	plt.imshow(img_bg_test,cmap=cmap_bg_test,interpolation='none')
	
	# plot
	cmap=plt.get_cmap('jet')
	plt.imshow(img,cmap=cmap,interpolation='none',vmin=zmin,vmax=zmax)
	plt.colorbar()
	
	plt.imsave("test.png",img)
	plt.show()
	plt.close()
	pass

def is_clockwise(polygon: shg.Polygon):
	xy=line_xy(polygon.boundary)
	x=xy[:,0]
	y=xy[:,1]
	diff_x=x[1:]-x[:-1]
	sum_y=y[:-1]+y[1:]
	area=np.sum(diff_x*sum_y)
	# logger.info('x:\n%s',x)
	# logger.info('y:\n%s',y)
	# logger.info('diff_x:\n%s',diff_x)
	# logger.info('sum_y:\n%s',sum_y)
	# logger.info('area: %s',area)
	if area>0:
		__isclockwise=True
	elif area<0:
		__isclockwise=False
	else:
		logger.info('self.polygon:\n%s',polygon)
		raise NotImplementedError('Area is zero.')
	return __isclockwise

def cut_polygon(polygon,cut_line):
	mpoly=polygon.difference(cut_line.buffer(1e-9))
	# logger.info('mpoly:\n%s',mpoly)
	return mpoly

def generate_cells(polygon,cutlines):
	if not cutlines: return [polygon]
	cells=list()
	# for i,cutline in enumerate(cutlines):
	if not polygon.intersects(cutlines[0]): return [polygon]
	mpoly=cut_polygon(polygon,cutlines[0])
	assert len(mpoly)>1
	for p in mpoly:
		cells.extend(generate_cells(p,cutlines[1:]))
	return cells

def get_from_geojson(filepath):
	geojson_data=geojson.load(open(filepath),)
	geoms=[shg.shape(x['geometry']).buffer(0) for x in geojson_data['features']]
	return geoms

def main():
	pass

if __name__=='__main__':
	main()
