import networkx as nx
import uuid
import itertools
from logging import INFO
import more_itertools as mi
from typing import Iterable
from sklearn.cluster import KMeans
from scipy import spatial
import shapely.ops as shops
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import collections
import shapely.geometry as shg
import lgblkb_tools.geometry as gmtr
from egistic_navigation.global_support import simple_logger
from egistic_navigation.katana_case import katana
from matplotlib import pyplot
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from numpy import asarray,concatenate,ones
from egistic_navigation import global_support as gsup
from lgblkb_tools.databases.sqla_orms import Cadastres_Info
from geoalchemy2.shape import to_shape

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

class TheLine(object):
	
	def __init__(self,coordinates=None,width=2.0,show='',line=None,text=None,**plot_kwargs):
		self.line=line or shg.LineString(coordinates=coordinates)
		self.width=width
		self.coverage=self.line.buffer(width,cap_style=shg.CAP_STYLE.round)
		self.__xy=None
		self.__slope=None
		self.__midpoint=None
		self.__perp_unit_vector=None
		self.__cover_line=None
		self.__show=show
		self.text=text
		
		if show: self.plot(show,**plot_kwargs)
	
	def plot(self,showcode='',show_self=1,show_normal=0,show_coverage=0,show_text=0,text=None,**kwargs):
		get_showcode=lambda:[int(x) for x in showcode]
		if showcode:
			if len(showcode)==1:
				show_self=int(showcode)
			elif len(showcode)==2:
				show_self,show_normal=get_showcode()
			elif len(showcode)==3:
				show_self,show_normal,show_coverage=get_showcode()
			elif len(showcode)==4:
				show_self,show_normal,show_coverage,show_text=get_showcode()
		if show_self:
			plt.plot(*self.line.xy,**dict(dict(),**kwargs))
		if show_normal:
			norm_line=self.get_normal_line(as_the_line=False)
			plt.plot(norm_line[:,0],norm_line[:,1],c='y')
		if show_coverage:
			gmtr.plot_patches([self.coverage],c='green',alpha=0.1)
		if show_text:
			# p1=plt.gca().transData.transform_point(self.xy[0])
			# p2=plt.gca().transData.transform_point(self.xy[1])
			# dy=abs(p2[1]-p1[1])
			# dx=abs(p2[0]-p1[0])
			# rotn=np.degrees(np.arctan2(dy,dx))
			trans_angle=plt.gca().transData.transform_angles(np.array((np.arctan(self.slope),)),self.midpoint.reshape((1,2)),
			                                                 radians=True)[0]
			a=np.array(self.line.interpolate(0.5,normalized=True).xy)
			a=a+(self.perp*3).reshape(a.shape)
			# print(a)
			# print((np.abs(self.perp)*5).reshape(a.shape))
			plt.annotate(text or self.text,a,rotation=trans_angle*180/np.pi,rotation_mode='anchor')
		return self
	
	def get_normal_line(self,as_the_line=True,width=None):
		normal_vector=self.perp*(width or self.width)
		normal_line=np.array([self.midpoint-normal_vector,self.midpoint+normal_vector])
		return TheLine(normal_line,width=self.width,show=self.__show) if as_the_line else normal_line
	
	def get_cover_line(self,cover_line_length):
		return self.get_normal_line().get_normal_line(width=cover_line_length)
	
	def get_field_lines(self,field_polygon,as_the_line=True):
		lines=field_polygon.polygon.intersection(self.get_cover_line(field_polygon.cover_line_length).line)
		if as_the_line:
			liner=lambda x:TheLine(line=x,width=self.width,show=self.__show)
		else:
			liner=lambda x:x
		
		if lines.is_empty: return None
		elif isinstance(lines,shg.MultiLineString):
			return [liner(x) for x in lines]
		else:
			return [liner(lines)]
	
	@property
	def xy(self):
		if self.__xy is None:
			self.__xy=np.array(self.line.xy).T
		return self.__xy
	
	@property
	def slope(self):
		if self.__slope is None:
			self.__slope=1/np.divide(*(self.xy[1,:]-self.xy[0,:]))
		return self.__slope
	
	@property
	def perp(self):
		if self.__perp_unit_vector is None:
			self.__perp_unit_vector=perpendicular(normalize(self.xy[1,:]-self.xy[0,:]))
		return self.__perp_unit_vector
	
	@property
	def midpoint(self):
		if self.__midpoint is None:
			self.__midpoint=np.sum(self.xy,0)/2
		return self.__midpoint
	
	def get_along_normal(self,distance,point_on_line=None):
		point_on_line=self.midpoint if point_on_line is None else point_on_line
		return point_on_line+self.perp*distance
	
	def offset_by(self,distance,count=1):
		offset_lines=list()
		for i in range(count):
			points=self.get_along_normal(distance=distance*(1+i),point_on_line=self.xy)
			line=TheLine(coordinates=points,width=self.width,show=self.__show)
			offset_lines.append(line)
		return offset_lines
	
	def __repr__(self):
		return "TheLine: "+str(self.line)

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

def line_xy(linestring):
	return np.array(linestring.xy).T

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
	# simple_logger.info('x:\n%s',x)
	# simple_logger.info('y:\n%s',y)
	# simple_logger.info('diff_x:\n%s',diff_x)
	# simple_logger.info('sum_y:\n%s',sum_y)
	# simple_logger.info('area: %s',area)
	if area>0:
		__isclockwise=True
	elif area<0:
		__isclockwise=False
	else:
		simple_logger.info('self.polygon:\n%s',polygon)
		raise NotImplementedError('Area is zero.')
	return __isclockwise

def cut_polygon(polygon,cut_line):
	mpoly=polygon.difference(cut_line.buffer(1e-9))
	# simple_logger.info('mpoly:\n%s',mpoly)
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

class ThePoly(object):
	
	def __init__(self,polygon):
		self.id=uuid.uuid4()
		self.polygon: shg.Polygon=polygon
		self.__xy=None
		self.__holes=None
		self.__cover_line_length=None
		self.__bounds_xy=None
		self.__lims=None
		self.__is_ok=None
		self.__isclockwise=None
		self.is_multilinestring=isinstance(self.polygon.boundary,shg.MultiLineString)
	
	def plot(self,text=None,**kwargs):
		gmtr.plot_polygon(self.polygon,**dict(dict(c='gray'),**kwargs))
		if text is not None: plt.text(*np.array(self.polygon.centroid.xy),s=text)
		return self
	
	def buffer(self,distance,resolution=16,quadsegs=None,cap_style=shg.CAP_STYLE.round,
	           join_style=shg.JOIN_STYLE.round,mitre_limit=5.0):
		return ThePoly(self.polygon.buffer(distance,resolution=resolution,quadsegs=quadsegs,
		                                   cap_style=cap_style,join_style=join_style,mitre_limit=mitre_limit))
	
	@property
	def holes(self):
		if self.__holes is None:
			self.__holes=[line_xy(x) for x in self.polygon.boundary[1:]]
		return self.__holes
	
	@property
	def all_xy(self):
		return np.concatenate([self.xy[:-1],*[x[:-1] for x in self.holes]])
	
	@property
	def xy(self):
		if self.__xy is None:
			if self.is_multilinestring:
				self.__xy=line_xy(self.polygon.boundary[0])
			else:
				self.__xy=line_xy(self.polygon.boundary)
		return self.__xy
	
	def get_lines(self,show=0):
		lines=list()
		current_chunk=self.polygon.boundary[0] if self.is_multilinestring else self.polygon.boundary
		for xy in self.xy[1:-1]:
			line,current_chunk=shops.split(current_chunk,shg.Point(xy))
			lines.append(line)
		lines.append(current_chunk)
		if show:
			for line in lines:
				plt.plot(*line.xy,lw=5)
		# plt.plot(*line.parallel_offset(5).xy)
		return lines
	
	@property
	def bounds_xy(self):
		if self.__bounds_xy is None:
			self.__bounds_xy=line_xy(self.polygon.envelope.boundary)
		return self.__bounds_xy
	
	@property
	def cover_line_length(self):
		if self.__cover_line_length is None:
			# xy=ThePoly(self.polygon.envelope).xy
			delta=self.bounds_xy[2]-self.bounds_xy[0]
			self.__cover_line_length=np.linalg.norm(delta)
		return self.__cover_line_length
	
	def get_field_lines(self,offset_distance,border_index=None,show=0):
		# if border_index is None: slicer=lambda x:x[:]
		# elif isinstance(border_index,Iterable): slicer=lambda x:[x[_i] for _i in border_index]
		# else: slicer=lambda x:x[border_index]
		if border_index is None: checker=lambda x:True
		elif isinstance(border_index,Iterable): checker=lambda x:x in border_index
		else: checker=lambda x:x==border_index
		borderlines=[TheLine(line=x,show='',width=offset_distance/2) for x in
		             self.buffer(-1e-9,join_style=shg.JOIN_STYLE.mitre).get_lines(show=0)]
		field_lines_data=collections.defaultdict(list)
		for i,borderline in enumerate(borderlines):
			if not checker(i): continue
			counter=0
			offset_lines=borderline.get_field_lines(self)
			offset_lines.extend(borderline.offset_by(offset_distance,int(np.ceil(self.cover_line_length/offset_distance))))
			offset_lines.extend(borderline.offset_by(-offset_distance,int(np.ceil(self.cover_line_length/offset_distance))))
			for offset_line in offset_lines:
				field_lines=offset_line.get_field_lines(self)
				if not field_lines:
					continue
				for field_line in field_lines:
					if not field_line: continue
					
					counter+=1
					field_lines_data[i].append(field_line)
			# if i==0 or i==1:
			# 	field_line.plot('100',c='magenta',alpha=0.5)
			simple_logger.info('counter: %s',counter)
		
		if show:
			fig=plt.gcf()
			fig.set_size_inches(16,9,forward=True)
			for i,b in enumerate(borderlines):
				if not checker(i): continue
				self.plot()
				boundary_label=f'Boundary_{i}'
				b.plot('1001',text=boundary_label)
				df=pd.DataFrame(field_lines_data[i],columns=[boundary_label])
				df['length']=df.applymap(lambda x:x.line.length)
				df[boundary_label].map(lambda x:x.plot('100',c='magenta',alpha=0.5))
		# df['length'].plot.hist()
		# plt.text(*np.array(self.polygon.centroid.xy),df.shape[0])
		# plt.show()
		return field_lines_data
	
	@property
	def lims(self):
		if self.__lims is None:
			self.__lims=np.array([self.bounds_xy[0],self.bounds_xy[2]])
		return self.__lims
	
	def pixelize(self,savename='',show=0,**fig_kwargs):
		plt.clf()
		fig=pyplot.figure(**dict(dict(num=1,figsize=(4,4),dpi=50,frameon=False),**fig_kwargs))
		ax=plt.Axes(fig,[0.,0.,1.,1.],frameon=False)
		ax.set_axis_off()
		fig.add_axes(ax)
		# ax=fig.add_subplot(111)
		
		path=pathify(self.polygon)
		# patch=PathPatch(path,facecolor='#cccccc',edgecolor='#999999')
		patch=PathPatch(path,facecolor='k',edgecolor='k')
		
		ax.add_patch(patch)
		lims=np.array([self.bounds_xy[0],self.bounds_xy[2]])
		ax.set_xlim(*lims[:,0])
		ax.set_ylim(*lims[:,1])
		ax.set_aspect(1.0)
		fig.canvas.draw()
		X=np.array(fig.canvas.renderer._renderer)
		if savename: fig.savefig(savename)
		plt.close(fig)
		if show:
			fig2=plt.figure()
			ax2=fig2.add_subplot(111,frameon=False)
			ax2.imshow(X)
			plt.show()
		return X
	
	def katanize(self,threshold,show=0,**kwargs):
		polygons=katana(self.polygon,threshold)
		if show:
			for p in polygons: gmtr.plot_polygon(p,**kwargs)
		return polygons
	
	def get_interior_points(self,distance,border_index=None,show=0):
		lines=collections.defaultdict(list)
		if border_index is None:
			field_box=ThePoly(self.polygon.envelope)
			for i,box_lines in field_box.get_field_lines(distance,[0,1]).items():
				for bl in box_lines:
					lines[i].append(bl)
		else:
			for i,field_lines in self.get_field_lines(distance,border_index).items():
				for bl in field_lines:
					lines[i].append(bl)
		the_data=pd.DataFrame(lines[(border_index[-1] if border_index is not None else 1)],columns=['line2s'])
		intersection_points=list()
		for l1 in lines[0]:
			points=the_data['line2s'].apply(lambda l2:l2.line.intersection(l1.line))
			not_empty_ones=points.apply(lambda x:isinstance(x,shg.Point))
			good_points=points[not_empty_ones]
			if good_points.empty: continue
			intersection_points.extend(good_points.tolist())
		ps=pd.Series(intersection_points)
		out=np.array(list(mi.flatten(ps.apply(lambda p:line_xy(p)))))
		if show:
			plt.scatter(out[:,0],out[:,1])
			pass
		return out
	
	def generate_clusters(self,resolution,n_clusters,show=''):
		interior_points=self.get_interior_points(distance=resolution)
		n_clusters=min(n_clusters,len(interior_points))
		simple_logger.info('n_clusters: %s',n_clusters)
		kmeans=KMeans(n_clusters=n_clusters,random_state=1)
		kmeans.fit(interior_points)
		if show:
			plot_clusters,plot_self=1,0
			if len(show)==1: plot_clusters=int(show)
			elif len(show)==2: plot_clusters,plot_self=[int(x) for x in show]
			if plot_self: self.plot()
			if plot_clusters:
				plt.scatter(kmeans.cluster_centers_[:,0],kmeans.cluster_centers_[:,1],c='k')
		return kmeans.cluster_centers_
	
	def to_numpy(self,resolution):
		simple_logger.info('self.lims: %s',self.lims)
		
		delta=self.lims[1]-self.lims[0]
		simple_logger.info('delta: %s',delta)
		pixel_count=np.ceil(delta/resolution)[::-1].astype(int)
		simple_logger.info('pixel_count: %s',pixel_count)
		img_array=np.zeros(shape=pixel_count)
		interior_points=np.round(self.get_interior_points(resolution),decimals=1)[:,[1,0]]
		positions_in_grid=np.floor(interior_points/resolution).astype(int)
		positions_in_grid=np.clip(positions_in_grid,a_min=[0,0],a_max=np.array(img_array.shape)-1)
		img_array[positions_in_grid[:,0],positions_in_grid[:,1]]=1
		return img_array
	
	@property
	def is_clockwise(self):
		if self.__isclockwise is not None: return self.__isclockwise
		x=self.xy[:,0]
		y=self.xy[:,1]
		diff_x=x[1:]-x[:-1]
		sum_y=y[:-1]+y[1:]
		area=np.sum(diff_x*sum_y)
		# simple_logger.info('x:\n%s',x)
		# simple_logger.info('diff_x:\n%s',diff_x)
		# simple_logger.info('y:\n%s',y)
		# simple_logger.info('sum_y:\n%s',sum_y)
		# simple_logger.info('area: %s',area)
		if area>0:
			self.__isclockwise=True
		elif area<0:
			self.__isclockwise=False
		else:
			simple_logger.info('self.polygon:\n%s',self.polygon)
			raise NotImplementedError('Area is zero.')
		return self.__isclockwise
	
	@property
	def is_ok(self):
		if self.__is_ok is not None: return self.__is_ok
		if not self.is_clockwise:
			simple_logger.info('self.polygon:\n%s',self.polygon)
			# simple_logger.info('"self" is clockwise, but should be counter-clockwise')
			simple_logger.info('"self" is counter-clockwise, but should be clockwise')
			self.__is_ok=False
			return self.__is_ok
		for interior in self.polygon.interiors:
			interior_poly=shg.Polygon(interior)
			if is_clockwise(interior_poly):
				simple_logger.info('Interior polygon:\n%s',interior_poly)
				simple_logger.info('"Interior polygon" is clockwise, but should be counter-clockwise')
				# simple_logger.info('"Interior polygon" is counter-clockwise, but should be clockwise')
				self.__is_ok=False
				return self.__is_ok
		self.__is_ok=True
		return self.__is_ok
	
	def __repr__(self):
		return "\n".join(["ThePoly",gsup.reprer(self)])
	
	def get_connectivity_lines_at(self,xy,**the_line_kwargs):
		line=TheLine([xy,xy+[0,1]])
		cover_line=line.get_cover_line(self.cover_line_length)
		res=cover_line.line.intersection(self.polygon)
		outs=list()
		if isinstance(res,shg.GeometryCollection):
			if isinstance(res[0],shg.Point): return []
			for geom in res:
				if isinstance(geom,shg.LineString) and geom.touches(shg.Point(*xy)):
					outs.append(TheLine(line=geom,**the_line_kwargs))
		elif isinstance(res,shg.MultiLineString):
			for geom in res:
				if geom.touches(shg.Point(*xy)):
					outs.append(TheLine(line=geom,**the_line_kwargs))
		return outs
	
	def get_boustrophedon_lines(self,**the_line_kwargs):
		outs=list()
		for xy in self.xy:
			lines=self.get_connectivity_lines_at(xy,**the_line_kwargs)
			if lines: outs.append(lines)
		for holexy in self.holes:
			xy_sorted=sorted(holexy[:-1],key=lambda x:x[0])
			for xy in [xy_sorted[0],xy_sorted[-1]]:
				lines=self.get_connectivity_lines_at(xy,**the_line_kwargs)
				if lines: outs.append(lines)
		return outs
	
	def get_boustrophedon_cells(self,as_the_poly=True,show=0,**plot_kwargs):
		assert self.is_ok
		line_groups=self.get_boustrophedon_lines()
		merged_lines=list()
		for lines in line_groups:
			line=shops.linemerge(shg.MultiLineString([x.line for x in lines]))
			merged_lines.append(line)
		merged_lines.sort(key=lambda x:x.xy[0])
		cells=generate_cells(self.polygon,merged_lines)
		simple_logger.info('len(cells): %s',len(cells))
		if show:
			for i,c in enumerate(cells):
				ThePoly(c.buffer(-0.5)).plot(**dict(dict(c=None,text=str(i)),**plot_kwargs))
		if as_the_poly: cells=[ThePoly(x) for x in cells]
		return cells
	
	def get_connectivity_graph(self,as_the_poly=True,show=''):
		G=nx.Graph()
		cells=self.get_boustrophedon_cells(as_the_poly=as_the_poly,show=int(show))
		
		G.add_nodes_from(cells)
		
		if as_the_poly: polymaker=lambda x:x.polygon
		else: polymaker=lambda x:x
		connections=list()
		bufferer=lambda x:x.buffer(1e-3,cap_style=shg.CAP_STYLE.square)
		
		for c1,c2 in itertools.combinations(cells,2):
			c1_poly: shg.Polygon=polymaker(c1)
			c2_poly: shg.Polygon=polymaker(c2)
			
			merged_mpoly=bufferer(shg.MultiPolygon([c1_poly,c2_poly]))
			if isinstance(merged_mpoly,shg.Polygon):
				intersection_area=bufferer(c1_poly).intersection(bufferer(c2_poly)).area
				if intersection_area<1e-5: continue
				connections.append([c1,c2])
				G.add_edge(c1,c2)
				if show:
					TheLine(line=shg.LineString([c1_poly.centroid,c2_poly.centroid]),
					        show=show,c='gray',ls='--')
		return cells,G

def check_cadastres():
	with gsup.mgr.session_context() as session:
		for i in gsup.infiterate(range(10000),inform_count=10):
			cad_info=session.query(Cadastres_Info).get(np.random.randint(4000000))
			if cad_info is None: continue
			cad_geom=gmtr.SpatialGeom(to_shape(cad_info.geom)).geom_obj
			for poly in cad_geom:
				try:
					assert ThePoly(poly).is_ok
				except:
					print(i)

def main():
	crop_field=ThePoly(shg.Polygon([
		[0,10],
		[10,100],
		[60,85],
		[120,120],
		[80,130],
		[200,150],
		[150,75],
		[200,0],
		],
		holes=[
			[
				[70.1,30.2],
				[100.4,20.3],
				[115.5,30.6],
				[90.4,30.6],
				[100.4,35.6],
				[80.8,40.7],
				
				],
			[
				[100,60],
				[145,60],
				[125,100],
				
				],
			]))  #.plot(c='k',lw=10)
	cells,graph=crop_field.get_connectivity_graph(True,show='000')
	target_cell=cells[0]
	target_cell.plot()
	print(target_cell)
	# target_cell.generate_clusters(1,50,'1')
	target_cell.get_interior_points(9,[0,1],1)
	plt.show()
	
	# plt.show()
	return
	graph_ids=collections.defaultdict(list)
	for cell1,cell2 in graph:
		graph_ids[cell1.id].append(cell2.id)
	for k,vs in graph_ids.items():
		print(len(vs))
	print(graph_ids)
	
	# crop_field.get_boustrophedon_cells(show=1)
	# crop_field.get_field_lines(9,[0,1],1)
	
	plt.show()
	
	# return
	
	return
	
	# print(mpoly)
	
	# plt.imshow(img)
	return
	# crop_field.get_field_lines(9,show=1)
	interior_points=crop_field.get_interior_points(1)
	# plt.plot(points[:,0],points[:,1])
	# target_distance=100
	# cluster_area=target_distance**2/4*np.pi
	# n_clusters=round(crop_field.polygon.area/cluster_area)
	n_clusters=3
	# simple_logger.info('n_clusters: %s',n_clusters)
	# filename=f'kmeans_{target_distance}.joblib'
	# if os.path.exists(filename) and False:
	# 	kmeans=joblib.load('kmeans.joblib')
	# else:
	kmeans=KMeans(n_clusters=n_clusters,random_state=1)
	kmeans.fit(interior_points)
	# joblib.dump(kmeans,filename)
	
	crop_field.plot()
	plt.scatter(kmeans.cluster_centers_[:,0],kmeans.cluster_centers_[:,1],c='k')
	plt.show()
	
	# crop_field.plot()
	# plt.show()
	return
	simple_logger.info('crop_field.lims:\n%s',crop_field.lims)
	simple_logger.info('crop_field.bounds_xy:\n%s',crop_field.bounds_xy)
	grid_size=10
	delta=crop_field.lims[1]-crop_field.lims[0]
	simple_logger.info('delta: %s',delta)
	# steps=delta/grid_size
	# simple_logger.info('steps: %s',steps)
	# ceiled_steps=np.ceil(steps)
	# simple_logger.info('ceiled_steps: %s',ceiled_steps)
	# corrected_grid_size=delta/ceiled_steps
	# simple_logger.info('corrected_grid_size: %s',corrected_grid_size)
	pixel_count=np.ceil(delta/grid_size)[::-1].astype(int)
	simple_logger.info('pixel_count: %s',pixel_count)
	img_array=np.zeros(shape=pixel_count)
	# simple_logger.info('img_array:\n%s',img_array)
	
	interior_points=np.round(crop_field.get_interior_points(grid_size),decimals=1)[:,[1,0]]
	# simple_logger.info('interior_points:\n%s',interior_points)
	positions_in_grid=np.floor_divide(interior_points,grid_size).astype(int)
	# simple_logger.info('positions_in_grid:\n%s',positions_in_grid)
	# positions_with_ones=np.ones((positions_in_grid.shape[0],positions_in_grid.shape[1]+1),dtype=int)
	# positions_with_ones[:,:-1]=positions_in_grid
	# simple_logger.info('positions_with_ones:\n%s',positions_with_ones)
	img_array[positions_in_grid[:,0],positions_in_grid[:,1]]=1
	print(img_array)
	# simple_logger.info('img_array:\n%s',img_array)
	# separate_img,cells=boustrophedon.bcd(img_array)
	# simple_logger.info('cells: %s',cells)
	
	return
	print(crop_field.all_xy)
	# crop_field=ThePoly(shg.Polygon([[0,0],[200,-10],[90,110],[10,100]]))
	borderlines=[TheLine(line=x,show='',width=target_distance/2) for x in
	             crop_field.buffer(-1e-9,join_style=shg.JOIN_STYLE.mitre).get_lines(show=0)]
	crop_field.plot()
	# vor=spatial.Voronoi(np.random.randint(0,100,(3,2)))
	tri=spatial.Delaunay(crop_field.all_xy)
	plt.triplot(crop_field.all_xy[:,0],crop_field.all_xy[:,1],tri.simplices)
	plt.plot(crop_field.all_xy[:,0],crop_field.all_xy[:,1],'o')
	# spatial.voronoi_plot_2d(vor,ax=plt.gca())
	plt.show()
	return

# return


# return

if __name__=='__main__':
	main()
