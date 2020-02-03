import collections
import functools
import gc
import itertools

import more_itertools as mi
import networkx as nx
import numpy as np
import pandas as pd
from box import Box
from lgblkb_tools import geometry as gmtr
from lgblkb_tools import logger
from matplotlib import pyplot as plt,pyplot
from matplotlib.patches import PathPatch
from shapely import geometry as shg,ops as shops
from sklearn.cluster import KMeans

from lgblkb_navigation.base_geometry.geom_utils import GenericGeometry,get_from_geojson,line_xy,pathify,is_clockwise,generate_cells,min_dist
from lgblkb_navigation.base_geometry.line_utils import TheLine
from lgblkb_navigation.base_geometry.point_utils import ThePoint
from lgblkb_navigation.katana_case import katana
from lgblkb_navigation.utils.google_way import SimpleTSP

class TheNodeView(object):

	def __init__(self,parent_graph,getter_obj):
		self.parent_graph=parent_graph
		self.getter_obj=getter_obj

	def __getitem__(self,item):
		if type(item) in [list,tuple]: return self.getter_obj(*item)
		else: return self.getter_obj(item)

class TheGraph(nx.Graph):

	#crop_field.G.nodes[decision_points['inner'][0]]['lines'][1].get_all_points
	def __init__(self,incoming_graph_data=None,**attr):
		super(TheGraph,self).__init__(incoming_graph_data=incoming_graph_data,**attr)
		self.decision_points=collections.OrderedDict()

	def _decision_points_getter(self,decision_key,node_index,*keys):
		result=self.nodes[self.decision_points[decision_key][node_index]]
		for key in keys:
			result=result[key]
		return result

	@property
	def inner_nodes(self):
		return TheNodeView(self,functools.partial(self._decision_points_getter,'inner'))

	@property
	def border_nodes(self):
		return TheNodeView(self,functools.partial(self._decision_points_getter,'border'))

class ThePoly(GenericGeometry):

	def __init__(self,polygon,**data):
		super(ThePoly,self).__init__(**data)

		if isinstance(polygon,shg.Polygon): self.polygon: shg.Polygon=polygon
		elif isinstance(polygon,ThePoly): self.polygon: shg.Polygon=polygon.polygon
		else: self.polygon: shg.Polygon=shg.Polygon(polygon)



		if not self.p.is_valid:
			# self.plot()
			# plt.show()
			# raise AssertionError(f"Polygon {self.polygon} is not a valid polygon.")
			# logger.warning(f"Polygon %s is not a valid polygon.",self.polygon)
			pass

		self._xy=None
		self._holes=None
		self._cover_line_length=None
		self._bounds_xy=None
		self._lims=None
		self._is_ok=None
		self._isclockwise=None
		if self.polygon.is_empty: self.is_multilinestring=False
		else: self.is_multilinestring=isinstance(self.polygon.boundary,shg.MultiLineString)

		self._is_convex=None
		self._convex_hull=None
		self._holes_xy=None
		self._holes=None

		# self.parent=parent
		self._exterior_lines=list()
		self._graph=None
		self._points=list()
		self._all_points=list()
		self._intersection_points=dict()
		self._all_lines=list()
		self._extension_lines=list()
		self._angles_at_vertices=None
		self.data=Box(data)

	@property
	def is_empty(self):
		return self.geometry.is_empty

	def get_simplified(self,tolerance=1e-9):
		return self.__class__(self.geometry.simplify(tolerance))

	@property
	def geometry(self):
		return self.polygon

	# region Class methods:
	@classmethod
	def from_tsp(cls,cities_count=10,seed=None,city_locator=None,**box_kwargs):
		np.random.seed(seed)
		if city_locator is None: city_locator=lambda x:x
		points=city_locator(np.random.rand(cities_count,2))
		stsp=SimpleTSP(points)
		route=stsp.run(depot=0,show=False)
		the_poly=cls(points[route],**box_kwargs)
		return the_poly

	@classmethod
	def synthesize(cls,cities_count,poly_extent=1000,hole_count=0,seed=None,hole_cities_count=None):
		if seed is None: seed=np.random.randint(0,int(1e6))
		logger.info('seed: %s',seed)
		np.random.seed(seed)
		try_count=0
		while True:
			try_count+=1
			logger.debug('Try main_poly: %s',try_count)
			seed=np.random.randint(100000000)
			try:
				unholy_field=cls.from_tsp(cities_count=cities_count,seed=seed,city_locator=lambda ps:ps*poly_extent)
				if hole_count<1: return unholy_field
				# holes_current_count=0
				holes=list()

				for i in range(1000):
					#.plot()
					seed=np.random.randint(100000000)
					hole=cls.from_tsp(cities_count=hole_cities_count or np.random.randint(4,10),seed=seed,
					                  city_locator=lambda ps:ps*np.random.randint(poly_extent*0.2,poly_extent*0.5)+
					                                         np.random.randint(poly_extent*0.1,poly_extent*0.2,2))
					# holy_poly=shg.Polygon(unholy_field.xy,[hole.xy,*map(lambda h:h.xy,holes)])
					# hole.plot()

					# logger.debug('unholy_field.p.contains(hole.p): %s',unholy_field.p.contains(hole.p))
					# logger.debug('unholy_field.p.intersects(hole.p): %s',unholy_field.p.intersects(hole.p))
					if not unholy_field.p.contains(hole.p): continue

					# 	logger.debug('Contains!!!')
					# if unholy_field.p.intersects(hole.p):
					# 	logger.debug('Intersects!!!')
					# if (not unholy_field.p.contains(hole.p)) or unholy_field.p.intersects(hole.p):
					# 	continue
					# hole.plot()
					# unholy_field.plot()
					# plt.show()
					hole_friendly=True
					for other_hole in holes:
						# logger.debug('other_hole.p.intersects(hole.p):\n%s',other_hole.p.intersects(hole.p))
						# logger.debug('other_hole.p.within(hole.p):\n%s',other_hole.p.within(hole.p))
						if other_hole.p.intersects(hole.p) or other_hole.p.within(hole.p):
							hole_friendly=False
							break
					if not hole_friendly: continue

					holes.append(hole)
					holy_poly=shg.Polygon(unholy_field.xy,list(map(lambda x:x.xy,holes)))  #.buffer(0)

					# if not holy_poly.is_valid:
					# 	holes.pop()
					# 	continue

					holy_field=cls(holy_poly,area=holy_poly.area)

					logger.debug('Proper hole created at i = %s',i)
					# holy_field.plot()
					# unholy_field.plot()
					# plt.show()

					if len(holy_field.holes)==hole_count:
						# if holes_current_count==hole_count:
						gc.collect()
						return holy_field
				gc.collect()
			# if holy_field.is_multilinestring:
			# 	# holes_current_count+=1
			# 	# logger.debug('holy_field:\n%s',holy_field)
			# 	# the_hole.plot()
			# 	# unholy_field.plot()
			# 	# plt.show()
			# 	# holes.append(the_hole)
			# 	# logger.debug('Hole %s added.',hole_count)
			# 	# holy_field=cls(shg.Polygon(unholy_field.xy,holes=list(map(lambda x:x.xy,holes))))#.plot(c='r')
			# 	if len(holy_field.holes)==hole_count:
			# 		# if holes_current_count==hole_count:
			# 		return holy_field
			except Exception as exc:
				logger.debug('exc:\n%s',exc)
				raise
				pass

	@classmethod
	def from_geojson(cls,filepath,**box_kwargs):
		return cls(get_from_geojson(filepath)[0],**box_kwargs)

	@classmethod
	def as_valid(cls,raw_polygon):
		raw_poly=ThePoly(raw_polygon)
		# for i, valid_point in enumerate(raw_poly.points):
		# 	valid_point.plot(f'P-{i}')
		invalid_point_indexes=list()
		for i,angle in enumerate(raw_poly.angles_at_vertices):
			# logger.debug('angle: %s',angle)
			if (180-angle)%180<min_dist:
				# logger.info('i: %s',i)
				invalid_point_indexes.append(i)
		valid_points=[x for i,x in enumerate(raw_poly.points) if not i in invalid_point_indexes]
		valid_poly=cls(valid_points)
		for hole in raw_poly.holes:

			valid_hole_poly=cls.as_valid(hole)
			try:
				diff_result=valid_poly.geometry.difference(valid_hole_poly.geometry)
				# if isinstance(diff_result,shg.MultiPolygon):
				# 	for diff_result_i in diff_result:
				# 		ThePoly(diff_result_i).plot()
				# 	plt.show()


			except:
				# # raw_poly.plot()
				# valid_poly.plot()
				# valid_hole_poly.plot('hole')
				# plt.show()

				# raise
				diff_result=valid_poly.geometry.buffer(1e-9).difference(valid_hole_poly.geometry)


			try:
				valid_poly=cls(diff_result)
			except:
				logger.warning('diff_result:\n%s',diff_result)
				valid_poly=cls(diff_result.buffer(1e-9))
				# raise


		# invalid_line_indexes.append(i-1)
		# for neighbor_index in [i-1,i+1]:
		# # neighbor_index=i-1
		# 	if raw_poly[i].angle-raw_poly[neighbor_index].angle<min_dist:
		# 		invalid_line_indexes.append(neighbor_index)
		# 		break

		# for i, valid_point in enumerate(valid_points):
		# 	valid_point.plot(f'P-{i}')

		#.plot(c='r')
		if len(valid_poly)!=len(raw_poly):
			return cls.as_valid(valid_poly)
		else:
			return valid_poly

	# endregion

	def original_poly(self):
		if not self.data.get('__toy_poly_scale__'):
			raise AttributeError('The polygon is not a toy. Original polygon does not exist.',dict(polygon=str(self)))
		return self.__class__(self.xy/self.data['__toy_poly_scale__']+self.data['__toy_poly_mean_xy__'])

	def toy_poly(self,scale=1e-3):
		mean_xy=np.mean(self.xy,axis=0)
		zero_mean_xy=self.xy-mean_xy
		# print(zero_mean_xy)
		scaled_xy=zero_mean_xy*scale
		crop_field=self.__class__(scaled_xy,__toy_poly_scale__=scale,__toy_poly_mean_xy__=mean_xy)  #.plot()
		return crop_field

	@property
	def p(self):
		return self.polygon

	def plot(self,text=None,**kwargs):
		# gmtr.plot_polygon(self.polygon,**dict(dict(c='gray'),**kwargs))
		gmtr.plot_polygon(self.polygon,**kwargs)
		gmtr.plot_patches(map(lambda h:h.p,self.holes),alpha=0.7)
		if text is not None: plt.text(*np.array(self.polygon.centroid.xy),s=text)
		return self

	def buffer(self,distance,resolution=16,quadsegs=None,cap_style=shg.CAP_STYLE.round,
	           join_style=shg.JOIN_STYLE.round,mitre_limit=5.0):
		return self.__class__(self.polygon.buffer(distance,resolution=resolution,quadsegs=quadsegs,
		                                          cap_style=cap_style,join_style=join_style,mitre_limit=mitre_limit))

	@property
	def reversed(self):
		return self.__class__(self.xy[::-1],**self.data)

	@property
	def holes(self):
		if self._holes is None:
			if isinstance(self.p.boundary,shg.MultiLineString):
				self._holes=[self.__class__(x,parent=self) for x in self.p.boundary[1:]]
			else:
				self._holes=[]
		return self._holes

	@property
	def holes_xy(self):
		if self._holes_xy is None:
			if isinstance(self.polygon.boundary,shg.MultiLineString):
				self._holes_xy=[line_xy(x) for x in self.polygon.boundary[1:]]
			else:
				self._holes_xy=[]
		return self._holes_xy

	@property
	def all_xy(self):
		return np.concatenate([self.xy[:-1],*[x[:-1] for x in self.holes_xy]])

	@property
	def xy(self):
		if self._xy is None:
			if self.is_multilinestring:
				self._xy=line_xy(self.polygon.boundary[0])
			else:
				self._xy=line_xy(self.polygon.boundary)
		return self._xy

	@property
	def bounds_xy(self):
		if self._bounds_xy is None:
			self._bounds_xy=line_xy(self.polygon.envelope.boundary)
		return self._bounds_xy

	@property
	def exterior_lines(self):
		if not self._exterior_lines: self._exterior_lines=self.get_exterior_lines()
		return self._exterior_lines

	@property
	def points(self):
		if not self._points:
			self._points=[ThePoint(xy) for xy in self.xy[:-1]]
		return self._points

	@property
	def all_points(self):
		if not self._all_points:
			self._all_points=[ThePoint(xy) for xy in self.all_xy]
		return self._all_points

	@property
	def all_lines(self):
		"""

		:rtype: [TheLine]
		"""
		if not self._all_lines:
			self._all_lines=[*self.exterior_lines,*mi.flatten([x.exterior_lines for x in self.holes])]
		return self._all_lines

	def get_exterior_lines(self,as_the_line=True,show=0):
		lines=list()
		if as_the_line: line_getter=lambda _line:TheLine(_line)
		else: line_getter=lambda _line:_line

		if self.is_multilinestring:
			is_ok=False
			for _ in self.polygon.boundary:
				is_ok=True
				break
			if not is_ok:
				logger.critical('self.geometry:\n%s',self.geometry)
				for p in self.points: p.plot()
				logger.critical('len(self.points): %s',len(self.points))
				plt.show()

		current_chunk=self.polygon.boundary[0] if self.is_multilinestring else self.polygon.boundary
		for xy in self.xy[1:-1]:
			# ThePoint(xy).plot()
			line,current_chunk=shops.split(current_chunk,shg.Point(xy))
			lines.append(line_getter(line))
		lines.append(line_getter(current_chunk))
		if show:
			for i,line in enumerate(lines):
				if as_the_line: line.plot()
				else: TheLine(line).plot()

		# TheLine(line=line).plot(**plot_kwargs)
		# logger.debug('line.xy: %s',line.xy)
		# plt.plot(*line.xy,**plot_kwargs)
		# plt.plot(*line.parallel_offset(5).xy)
		return lines

	@property
	def cover_line_length(self):
		if self._cover_line_length is None:
			# xy=self.__class__(self.polygon.envelope).xy
			delta=self.bounds_xy[2]-self.bounds_xy[0]
			self._cover_line_length=np.linalg.norm(delta)*1.01
		return self._cover_line_length

	# def get_field_lines(self,offset_distance,border_index=None,show=0):
	# 	# if border_index is None: slicer=lambda x:x[:]
	# 	# elif isinstance(border_index,Iterable): slicer=lambda x:[x[_i] for _i in border_index]
	# 	# else: slicer=lambda x:x[border_index]
	# 	if border_index!=0 and not border_index: checker=lambda x:True
	# 	elif isinstance(border_index,Iterable): checker=lambda x:x in border_index
	# 	else: checker=lambda x:x==border_index
	# 	borderlines=[TheLine(x,width=offset_distance/2) for x in
	# 	             self.buffer(-1e-8,join_style=shg.JOIN_STYLE.mitre).get_exterior_lines(as_the_line=False,show=0)]
	# 	field_lines_data=collections.defaultdict(list)
	# 	for i,borderline in enumerate(borderlines):
	# 		if not checker(i): continue
	# 		counter=0
	# 		offset_lines=borderline.get_field_lines(self)
	# 		offset_lines.extend(borderline.offset_by(offset_distance,int(np.ceil(self.cover_line_length/offset_distance))))
	# 		offset_lines.extend(borderline.offset_by(-offset_distance,int(np.ceil(self.cover_line_length/offset_distance))))
	#
	# 		for offset_line in offset_lines:
	# 			# offset_line.plot()
	#
	# 			field_lines=offset_line.get_field_lines(self)
	# 			if not field_lines:
	# 				continue
	# 			for field_line in field_lines:
	# 				if not field_line: continue
	#
	# 				counter+=1
	# 				field_lines_data[i].append(field_line)
	# 	# plt.show()
	# 	# if i==0 or i==1:
	# 	# 	field_line.plot('100',c='magenta',alpha=0.5)
	# 	# logger.info('counter: %s',counter)
	# 	fl_counters=[len(x) for x in field_lines_data.values()]
	# 	logger.debug('# of field lines along borders: %s',fl_counters)
	#
	# 	if show:
	# 		# fig=plt.gcf()
	# 		# fig.set_size_inches(16,9,forward=True)
	# 		for i,b in enumerate(borderlines):
	# 			if not checker(i): continue
	# 			# self.plot()
	# 			boundary_label=f'Boundary_{i}'
	# 			b.plot('1001',text=boundary_label)
	# 			# df=pd.DataFrame(field_lines_data[i],columns=[boundary_label])
	# 			# df['length']=df.applymap(lambda x:x.line.length)
	# 			# df[boundary_label].map(lambda x:x.plot('100',c='magenta',alpha=0.5))
	# 			for field_line in field_lines_data[i]:
	# 				field_line.plot()
	#
	# 	# df['length'].plot.hist()
	# 	# plt.text(*np.array(self.polygon.centroid.xy),df.shape[0])
	# 	# plt.show()
	# 	return field_lines_data

	@property
	def lims(self):
		if self._lims is None:
			self._lims=np.array([self.bounds_xy[0],self.bounds_xy[2]])
		return self._lims

	# region Discretization methods:
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
		ax.set_aspect('1.0')
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

	@logger.trace()
	def katanize(self,threshold,show=0,**kwargs):
		polygons=katana(self.polygon,threshold)
		if show:
			for p in polygons: gmtr.plot_polygon(p,**kwargs)
		return polygons

	# def get_interior_points(self,distance,border_index=None,show=0):
	# 	lines=collections.defaultdict(list)
	# 	if border_index is None:
	# 		# field_box=ThePoly(self.polygon.envelope)
	# 		for i,box_lines in self.get_field_lines(distance,[0,1]).items():
	# 			for bl in box_lines:
	# 				lines[i].append(bl)
	# 	else:
	# 		for i,field_lines in self.get_field_lines(distance,border_index).items():
	# 			for bl in field_lines:
	# 				lines[i].append(bl)
	# 	the_data=pd.DataFrame(lines[(border_index[-1] if border_index is not None else 1)],columns=['line2s'])
	# 	intersection_points=list()
	# 	for l1 in lines[0]:
	# 		points=the_data['line2s'].apply(lambda l2:l2.line.intersection(l1.line))
	# 		not_empty_ones=points.apply(lambda x:isinstance(x,shg.Point))
	# 		good_points=points[not_empty_ones]
	# 		if good_points.empty: continue
	# 		intersection_points.extend(good_points.tolist())
	# 	ps=pd.Series(intersection_points)
	# 	out=np.array(list(mi.flatten(ps.apply(lambda p:line_xy(p)))))
	# 	if show:
	# 		plt.scatter(out[:,0],out[:,1])
	# 		pass
	# 	return out

	@logger.trace()
	def generate_clusters(self,grid_resolution,n_clusters,show='',random_state=1):
		# interior_points=self.get_interior_points(distance=grid_resolution)
		interior_points=self.generate_grid_points(grid_resolution=grid_resolution)
		logger.debug('len(interior_points): %s',len(interior_points))
		n_clusters=min(n_clusters,len(interior_points))
		logger.debug('n_clusters: %s',n_clusters)
		kmeans=KMeans(n_clusters=n_clusters,random_state=random_state)
		kmeans.fit(interior_points)
		if show:
			plot_clusters,plot_self=1,0
			if len(show)==1: plot_clusters=int(show)
			elif len(show)==2: plot_clusters,plot_self=[int(x) for x in show]
			if plot_self: self.plot()
			if plot_clusters:
				plt.scatter(kmeans.cluster_centers_[:,0],kmeans.cluster_centers_[:,1],c='k')
		return kmeans.cluster_centers_

	@logger.trace()
	def generate_grid_points(self,grid_resolution):
		corners=self.bounds_xy[[0,2]]
		x_grid_info=np.arange(*corners[:,0],step=grid_resolution)
		y_grid_info=np.arange(*corners[:,1],step=grid_resolution)
		grid_x,grid_y=np.meshgrid(x_grid_info,y_grid_info)
		xy_coors=np.stack([grid_x.ravel(),grid_y.ravel()]).T
		# print(xy_coors)
		xy_df=pd.DataFrame(xy_coors,columns=['x','y'])
		xy_df['inside']=xy_df.apply(lambda coor:self.polygon.contains(shg.Point([coor.x,coor.y])),axis=1)
		xy_df=xy_df[xy_df['inside']].drop(columns=['inside'])
		logger.debug('xy_df.shape: %s',xy_df.shape)
		return xy_df

	def to_numpy(self,resolution):
		logger.info('self.lims: %s',self.lims)

		delta=self.lims[1]-self.lims[0]
		logger.info('delta: %s',delta)
		pixel_count=np.ceil(delta/resolution)[::-1].astype(int)
		logger.info('pixel_count: %s',pixel_count)
		img_array=np.zeros(shape=pixel_count)
		interior_points=np.round(self.generate_grid_points(resolution),decimals=1)[:,[1,0]]
		positions_in_grid=np.floor(interior_points/resolution).astype(int)
		positions_in_grid=np.clip(positions_in_grid,a_min=[0,0],a_max=np.array(img_array.shape)-1)
		img_array[positions_in_grid[:,0],positions_in_grid[:,1]]=1
		return img_array

	# endregion

	# region Booleans:
	@property
	def is_clockwise(self):
		if self._isclockwise is not None: return self._isclockwise
		x=self.xy[:,0]
		y=self.xy[:,1]
		diff_x=x[1:]-x[:-1]
		sum_y=y[:-1]+y[1:]
		area=np.sum(diff_x*sum_y)
		# logger.info('x:\n%s',x)
		# logger.info('diff_x:\n%s',diff_x)
		# logger.info('y:\n%s',y)
		# logger.info('sum_y:\n%s',sum_y)
		# logger.info('area: %s',area)
		if area>0:
			self._isclockwise=True
		elif area<0:
			self._isclockwise=False
		else:
			logger.warning('self.polygon:\n%s',self.polygon)
			# self._isclockwise=None
			self.plot()
			plt.show()
			raise NotImplementedError('Area is zero.')
		return self._isclockwise

	@property
	def is_ok(self):
		if self._is_ok is not None: return self._is_ok
		if not self.is_clockwise:
			logger.info('self.polygon:\n%s',self.polygon)
			# logger.info('"self" is clockwise, but should be counter-clockwise')
			logger.info('"self" is counter-clockwise, but should be clockwise')
			self._is_ok=False
			return self._is_ok
		for interior in self.polygon.interiors:
			interior_poly=shg.Polygon(interior)
			if is_clockwise(interior_poly):
				logger.info('Interior polygon:\n%s',interior_poly)
				logger.info('"Interior polygon" is clockwise, but should be counter-clockwise')
				# logger.info('"Interior polygon" is counter-clockwise, but should be clockwise')
				self._is_ok=False
				return self._is_ok
		self._is_ok=True
		return self._is_ok

	def touches(self,xy):
		return self.polygon.touches(shg.Point(xy))

	def as_noncollinear(self):
		non_collinear_points=list()
		for i_vertex,angle_at_vertex in enumerate(self.angles_at_vertices):
			if abs(angle_at_vertex-180)<min_dist: continue
			non_collinear_points.append(self.points[i_vertex])
		non_collinear_field=self.__class__(non_collinear_points)
		return non_collinear_field

	@property
	def is_convex(self):
		if self._is_convex is None:
			self._is_convex=True
			for angle_at_vertex in self.angles_at_vertices:
				if angle_at_vertex>180+min_dist:
					self._is_convex=False
					break
		# if abs(self.p.area-self.convex_hull.p.area)<min_dist:
		# 	self._is_convex=True
		# else:
		# 	self._is_convex=False
		return self._is_convex

	# endregion

	# region Boustrophedon algorithm:
	def get_connectivity_lines_at(self,xy,**the_line_kwargs):
		line=TheLine([xy,xy+[0,1]])
		cover_line=line.get_cover_line(self.cover_line_length)
		res=cover_line.line.intersection(self.polygon)
		outs=list()
		if isinstance(res,shg.GeometryCollection):
			if isinstance(res[0],shg.Point): return []
			for geom in res:
				if isinstance(geom,shg.LineString) and geom.touches(shg.Point(*xy)):
					outs.append(TheLine(geom,**the_line_kwargs))
		elif isinstance(res,shg.MultiLineString):
			for geom in res:
				if geom.touches(shg.Point(*xy)):
					outs.append(TheLine(geom,**the_line_kwargs))
		return outs

	def get_boustrophedon_lines(self,**the_line_kwargs):
		outs=list()
		for xy in self.xy:
			lines=self.get_connectivity_lines_at(xy,**the_line_kwargs)
			if lines: outs.append(lines)
		for holexy in self.holes_xy:
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
		logger.info('len(cells): %s',len(cells))
		if show:
			for i,c in enumerate(cells):
				self.__class__(c.buffer(-0.5)).plot(**dict(dict(c=None,text=str(i)),**plot_kwargs))
		if as_the_poly: cells=[self.__class__(x) for x in cells]
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
					TheLine(shg.LineString([c1_poly.centroid,c2_poly.centroid])).plot(c='gray',ls='--')
		return cells,G

	# endregion

	# def get_convex_hull(self):
	# 	if self._convex_hull is None:
	# 		hull=spatial.ConvexHull(self.xy)
	# 		vertices=np.array(sorted(hull.vertices))
	# 		# convex_poly=ThePoly(hull.points[hull.vertices]).plot(c='b',alpha=0.5)
	# 		convex_poly=ThePoly(self.xy[vertices])  #.plot(c='b',alpha=0.5)
	# 		self._convex_hull=convex_poly
	# 	return self._convex_hull
	@property
	def convex_hull(self):
		if self._convex_hull is None:
			# logger.debug('self.xy:\n%s',self.xy)
			if self.p.convex_hull.is_empty:
				# self.plot()
				# plt.show()
				logger.warning('Empty hull')
				self._convex_hull=ThePoly(shg.Polygon())
			else:
				self._convex_hull=ThePoly(self.p.convex_hull,**self.data)
		return self._convex_hull

	def __contains__(self,item):
		if isinstance(item,ThePoint):
			return item in self.points
		elif isinstance(item,ThePoly):
			return self.geometry.contains(item.geometry) or self.geometry.intersects(item.geometry) or self.geometry.touches(item.geometry)

	def __eq__(self,other):
		# if isinstance(other,ThePoly):
		# 	return self.id==other.id
		# else:
		# 	return self.id==ThePoly(other).id
		# is_equal=True
		if isinstance(other,ThePoly):
			other_points=other.points
		else:
			other_points=ThePoly(other).points

		other_points_copy=other_points[:]
		for point in self.points:
			if not point in other_points_copy:
				return False
			other_points_copy.remove(point)
		if other_points_copy: return False
		return True

	def __len__(self):
		return len(self.xy)-1

	def _add_node(self,G,xy,parent_lines,is_intermediate=False):
		node=ThePoint(xy)
		l1,l2=parent_lines
		if is_intermediate:
			if not node in l1:
				l1.intermediate_points[node.associate_with_lines(l1,l2)]=l2
			if not node in l2:
				l2.intermediate_points[node.associate_with_lines(l1,l2)]=l1
		G.add_node(node,lines=parent_lines)
		return node

	def _add_edge(self,G,line):
		G.add_edge(line[0],line[1],line=line)
		return line

	# @logger.trace()
	def generate_graph(self,parent_graph=None):
		G=parent_graph or TheGraph()
		# vertex_angles=self.get_angles_at_vertices()
		# if self.parent: vertex_angles=360-vertex_angles

		# point_kwargs=dict()
		# point_kwargs.update(box_kwargs)
		# point_kwargs.update(point_opts or {})

		for i in range(len(self.exterior_lines)):
			# next_index=(i+1) if i+1==len(lines) else 0
			# node=ThePoint(self.xy[i])
			# G.add_node(node,lines=[lines[i],lines[i-1]])
			self._add_node(G,self.xy[i],parent_lines=[self.exterior_lines[i],self.exterior_lines[i-1]])
			# next_node=ThePoint(self.xy[next_index])
			# G.add_edge(node,next_node,line=lines[i])
			pass
			# self._add_edge(G,self.exterior_lines[i])
			pass

		for hole in self.holes:
			hole.generate_graph(G)

		# for i, xy in enumerate(self.xy):
		# 	G.add_node(i,)

		# for i,line in enumerate(lines):
		# 	# G.add_edge(*map(ThePoint,line.xy),line=line)
		# 	if i+1==len(lines):
		# 		G.add_edge(i,0,line=line)
		# 	else:
		# 		G.add_edge(i,i+1,line=line)
		return G

	@property
	def G(self):
		if self._graph is None: self._graph=self.generate_graph()
		return self._graph

	def generate_intersection_points(self,lines):
		intersection_points=list()
		for l1,l2 in itertools.combinations(lines,2):
			if l1[0]==l2[0]: continue
			result=l1.line.intersection(l2.line)
			if result.is_empty: continue
			the_point=ThePoint(result)
			self._add_node(self.G,the_point,[l1,l2],is_intermediate=True)
			# self.G.add_node(the_point,lines=[l1,l2])
			intersection_points.append(the_point)
		# intersection_points=set(intersection_points)
		logger.info('intersection_points count: %s',len(intersection_points))
		return intersection_points

	def get_angles_at_vertices(self):
		vectors=self.xy[1:]-self.xy[:-1]
		line_angles=np.arctan2(vectors[:,1],vectors[:,0])*180/np.pi
		line_angles=np.append(line_angles,line_angles[0])
		angle_diff=((line_angles[1:]-line_angles[:-1])+180)%360
		angle_at_vertices=np.zeros(angle_diff.shape)
		angle_at_vertices[0]=angle_diff[-1]
		angle_at_vertices[1:]=angle_diff[:-1]
		if self.is_clockwise:
			return angle_at_vertices
		else:
			return 360-angle_at_vertices

	@property
	def angles_at_vertices(self):
		if self._angles_at_vertices is None:
			self._angles_at_vertices=self.get_angles_at_vertices()
		return self._angles_at_vertices

	def __getitem__(self,item):
		# logger.debug('item: %s',item)
		if type(item) in (np.ndarray,tuple,list):
			# if isinstance(item,np.ndarray): item=item.ravel()
			u,v=item
			return self.G.nodes[u,v]
		elif isinstance(item,slice):
			return [self[ii] for ii in range(*item.indices(len(self)))]
		elif isinstance(item,int):
			return self.exterior_lines[item]
		else:
			return super().__getitem__(item)

class TheChunk(ThePoly):

	def __init__(self,polygon,parent,**data):
		super(TheChunk,self).__init__(polygon=polygon,**data)
		self.parent: ThePoly=parent
		self.touches_parent=self.parent.p.exterior.touches(self.p)  # if parent else False
		self.baseline=None
		if self.touches_parent:
			parent_lines=self.parent.get_exterior_lines()
			for chunk_line in self.get_exterior_lines():
				if chunk_line not in parent_lines:
					self.baseline=chunk_line
					# chunk_line.plot(text='baseline')
					break

			assert self.baseline is not None
