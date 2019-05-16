import networkx as nx
import numpy as np
import shapely.geometry as shg

from egistic_navigation.base_geometry.geom_utils import min_dist
from egistic_navigation.base_geometry.line_utils import TheLine
from egistic_navigation.base_geometry.point_utils import ThePoint
from egistic_navigation.base_geometry.poly_utils import ThePoly
from egistic_navigation.global_support import simple_logger,with_logging

class DecisionPoint(ThePoint):
	
	def __init__(self,xy,**box_kwargs):
		super(DecisionPoint,self).__init__(xy=xy,**box_kwargs)
	
	pass

class TheField(ThePoly):
	
	def __init__(self,polygon,**box_kwargs):
		super(TheField,self).__init__(polygon=polygon,**box_kwargs)
		self.__graph=None
		self.__extension_lines=list()
	
	def get_graph(self,parent_graph=None,**box_kwargs):
		G=parent_graph or nx.Graph()
		lines=self.get_exterior_lines(**box_kwargs)
		for i in range(len(lines)):
			next_index=(i+1) if i+1==len(lines) else 0
			node=ThePoint(self.xy[i],parent=self,is_exterior=True,**box_kwargs)
			next_node=ThePoint(self.xy[next_index],parent=self,**box_kwargs)
			# if nodes_as_the_points:
			# 	node=ThePoint(self.xy[i],i)
			# 	next_node=ThePoint(self.xy[next_index],next_index)
			# else:
			# 	node=i
			# 	next_node=next_index
			G.add_node(node,lines=[lines[i],lines[i-1]])
			G.add_edge(node,next_node,line=lines[i])
		
		for hole in self.holes:
			hole.get_graph(G,)
		
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
	def graph(self):
		if self.__graph is None: self.__graph=self.get_graph()
		return self.__graph
	
	@with_logging()
	def get_extension_lines(self,parent=None,show=False):
		the_main_poly=parent or self  #ThePoly(self.p)#self
		if self.is_convex:
			simple_logger.info('The field is convex. Extension lines are absent.')
			return self
		
		delta=self.convex_hull.p.difference(self.p)
		if delta.is_empty:
			simple_logger.info('Delta is empty. Skipping.')
			return self
		elif isinstance(delta,shg.MultiPolygon):
			chunks=[TheChunk(x,self,id=f'Chunk-{i}') for i,x in enumerate(delta)]
		elif isinstance(delta,shg.Polygon):
			chunks=[TheChunk(delta,self)]
		else:
			simple_logger.critical('delta: %s',delta)
			# self.plot()
			# self.convex_hull.plot(c='b')
			# plt.show()
			raise NotImplementedError
		
		out_extension_lines=list()
		for i_chunk,the_chunk in enumerate(chunks):
			# if i_chunk!=3: continue
			chunk_hull=the_chunk.convex_hull  #.plot(c='b',lw=0.5,ls='--',text=f'chunk_hull-{i_chunk}')
			for i_point,xy in enumerate(the_chunk.xy[:-1]):
				xy_point=ThePoint(xy,)  #.plot(f'{i_point}')
				# simple_logger.debug('Point-%s',i_point)
				# the_chunk.baseline.plot(c='r')
				if not the_chunk.touches_parent or (chunk_hull.touches(xy) and the_chunk.baseline.line.distance(xy_point.g)>min_dist):
					# simple_logger.info('Point-%s',i_point)
					# extension_lines=the_chunk[xy]['extension_lines']=the_main_poly[xy]['extension_lines']=list()
					extension_lines=the_main_poly[xy]['extension_lines']=list()
					for i_line,line in enumerate(the_chunk[xy]['lines']):
						line: TheLine=line  #.plot(text=str(i_line))
						extension_line=line.extend_from(xy,the_main_poly.cover_line_length)
						cropped_line=the_main_poly.p.intersection(extension_line.line)
						# simple_logger.debug('cropped_line:\n%s',cropped_line)
						if isinstance(cropped_line,shg.MultiLineString):
							cropped_line=cropped_line[0]
						elif isinstance(cropped_line,shg.GeometryCollection):
							continue
						
						# self.plot()
						cropped_extension_line=TheLine(cropped_line)  #.plot()
						# xy_point.plot(i_point)
						# cropped_extension_line.plot()
						# plt.show()
						if xy_point in cropped_extension_line:
							if cropped_extension_line.line.length<1e-3:
								simple_logger.warning('Too short extension line encountered.')
							extension_lines.append(cropped_extension_line)
							out_extension_lines.append(cropped_extension_line)
			
			geom_collection=chunk_hull.p.intersection(self.p)
			sub_chunks=list()
			for geom_obj in geom_collection:
				if isinstance(geom_obj,shg.LineString):
					# TheLine(geom_obj).plot(text='line',c='y')
					pass
				elif isinstance(geom_obj,shg.MultiLineString):
					# [TheLine(x).plot(text='line',c='cyan') for x in geom_obj]
					pass
				elif isinstance(geom_obj,shg.Polygon):
					sub_chunk=TheField(geom_obj)  #.plot(text='ThePoly',c='r')
					sub_chunks.append(sub_chunk)
				else:
					simple_logger.debug('geom_obj: %s',geom_obj)
					raise NotImplementedError
			
			for smaller_chunk in sub_chunks:
				if not smaller_chunk.is_convex:
					# the_chunk.plot(f'the_chunk-{i_chunk}')
					# smaller_chunk.plot('smaller_chunk',c='r',lw=0.3)
					# smaller_chunk.id="_".join(map(str,[self.id,'smaller_chunk']))
					# simple_logger.debug('smaller_chunk:\n%s',smaller_chunk)
					smaller_chunk.get_extension_lines(parent=the_main_poly)
		
		if show:
			for ext_line in out_extension_lines:
				ext_line.plot()
		simple_logger.debug('Extension lines count: %s',len(out_extension_lines))
		return out_extension_lines
	
	@property
	def extension_lines(self):
		if not self.__extension_lines:
			self.__extension_lines=self.get_extension_lines()
		return self.__extension_lines
	
	def __getitem__(self,item):
		# simple_logger.debug('item: %s',item)
		if type(item) in (np.ndarray,tuple,list):
			if isinstance(item,np.ndarray): item=item.ravel()
			u,v=item
			return self.graph.nodes[u,v]
		elif isinstance(item,int):
			return self.graph.nodes[item]
		else:
			return super().__getitem__(item)

class TheChunk(TheField):
	
	def __init__(self,polygon,parent=None,**box_kwargs):
		super(TheChunk,self).__init__(polygon=polygon,**box_kwargs)
		self.parent: TheField=parent
		self.touches_parent=self.parent.p.exterior.touches(self.p) if parent else False
		self.baseline=None
		if self.touches_parent:
			parent_lines=self.parent.get_exterior_lines()
			for chunk_line in self.get_exterior_lines():
				if chunk_line not in parent_lines:
					self.baseline=chunk_line
					# chunk_line.plot(text='baseline')
					break
			assert self.baseline is not None

@with_logging()
def main():
	# tamasha()
	# return
	# # cad_info=gsup.mgr.query_get(Cadastres_Info,1516792).to_dump(gsup.project_folder['data'],'cad_info')
	# # cad_info=Cadastres_Info.from_dump(gsup.project_folder['data'],'cad_info')
	# # cad_info=cad_info.as_dumped()
	# # simple_logger.debug('cad_info:\n%s',cad_info)
	# # sg=gmtr.SpatialGeom(to_shape(cad_info.geom)).convert_crs(4326,3857)
	#
	# # crop_field=ThePoly(sg.geom_obj[0])#.plot(c='b')
	# # simple_logger.debug('crop_field.polygon.area:\n%s',crop_field.polygon.area)
	# # crop_field=ThePoly(shg.Polygon(vw.simplify(crop_field.xy,threshold=crop_field.polygon.area/200))).plot(c='green')
	# # simple_logger.debug('crop_field.polygon.area:\n%s',crop_field.polygon.area)
	# # lines=crop_field.get_exterior_lines()
	# # for line in lines:
	# # 	line.plot()
	# # simple_logger.debug('len(lines): %s',len(lines))
	# # plt.show()
	#
	# return
	# # coverage_width=24
	# # filepath=r'/home/lgblkb/cft.geojson'
	# # ax=plt.subplot()
	# # crop_field=ThePoly(get_from_geojson(filepath)[0]).plot()
	# # crop_field.xy
	# return
	# # point_count=int(crop_field.polygon.area/coverage_width**2)
	# # cluster_centers=crop_field.generate_clusters(50,100,show='1')
	# # cluster_centers_path=gsup.project_folder['data']['cluster_centers.joblib']
	#
	# # joblib.dump(cluster_centers,cluster_centers_path)
	#
	# # plt.show()
	# return
	# # vor=Voronoi(cluster_centers)
	# # voronoi_plot_2d(vor,ax=ax)
	# # crop_field.plot(ax=ax)
	# # plt.xlim(crop_field.bounds_xy[[0,2]][:,0])
	# # plt.ylim(crop_field.bounds_xy[[0,2]][:,1])
	# # plt.show()
	# pass
	# # print(crop_field.all_xy)
	# # plt.show()
	# return
	# pass
	# # l1=TheLine([[0,0],[2,2]])
	# # l2=TheLine([[0,2],[2,0]])
	# # split_point=l1.line.intersection(l2.line)
	# # a=shops.split(l1.line,split_point)
	# # simple_logger.debug('a:\n%s',a)
	# #
	#
	# pass
	# # return
	# # return
	# # holes_xy=[
	# # 	[
	# # 		[70.1,30.2],
	# # 		[100.4,20.3],
	# # 		[115.5,30.6],
	# # 		[90.4,30.6],
	# # 		[100.4,35.6],
	# # 		[80.8,40.7],
	# #
	# # 		],
	# # 	[
	# # 		[100,60],
	# # 		[145,60],
	# # 		[125,100],
	# #
	# # 		],
	# # 	]
	# # crop_field=ThePoly(shg.Polygon([
	# # 	[0,10],
	# # 	[10,100],
	# # 	[60,85],
	# # 	[120,120],
	# # 	[80,130],
	# # 	[200,150],
	# # 	[150,75],
	# # 	[200,0],
	# # 	],)).plot(c='k',lw=1)
	# # crop_field=ThePoly(shg.Polygon([
	# # 	[0,10],
	# # 	[10,100],
	# # 	[60,85],
	# # 	[120,120],
	# # 	[80,130],
	# # 	[200,150],
	# # 	[150,75],
	# # 	[200,0],
	# # 	],)).plot(c='k',lw=1)
	# # # print(crop_field.polygon.area)
	# # # print(crop_field.xy)
	# # # crop_field.generate_grid_points(grid_resolution=1)
	# # # crop_field.generate_clusters(1,100,show='1')
	# # # print(shg.Polygon(crop_field.polygon.exterior).buffer(-9))#
	# # # return
	# # # level_1=ThePoly(shg.Polygon(crop_field.polygon.exterior,holes_xy=[shg.Polygon(crop_field.polygon.exterior).buffer(-9).exterior])).plot()
	# # # level_1.generate_clusters(0.5,92,'1')
	# # # print(level_1.polygon.exterior.length)
	# pass
	#
	# # lines_data: dict=crop_field.get_field_lines(500,border_index=[],show=0)
	# # simple_logger.debug('lines_data:\n%s',lines_data)
	# pass
	#
	# # simple_logger.debug('target_border_index: %s',target_border_index)
	# # simple_logger.debug('other_border_indexes: %s',other_border_indexes)
	# # other_lines=list()
	# # for other_border_index in other_border_indexes:
	# # 	other_lines.extend(lines_data[other_border_index])
	#
	# # lines=list()
	# # for k,vs in lines_data.items():
	# # 	for vi in vs:
	# # 		lines.append([k,vi])
	# # df_lines=pd.DataFrame(lines,columns=['border_index','the_line'])
	# # partitioner=lambda key:[list(x) for x in mi.partition(lambda k:k!=key,lines_data.keys())]
	# # target_border_indexes,other_border_indexes=partitioner(0)
	# # target_border_index=target_border_indexes[0]
	# # target_lines=lines_data[target_border_index]
	#
	# # indexes=[x for x in mi.flatten(partitioner(0))]
	# # the_lines=[x.line for x in mi.flatten(lines_data[x] for x in indexes)]
	# # a=shops.cascaded_union(the_lines)
	# # a=shops.cascaded_union([x.line for x in [lines_data[target_border_index][0],*other_lines]])
	# # a=shops.cascaded_union(df_lines['the_line'].map(lambda tl:tl.line).tolist())
	# # simple_logger.debug('a:\n%s',a)
	# # return
	# # for line in a:
	# # 	TheLine(line).plot()
	# # # print(line)
	# # crop_field.plot()
	# # plt.show()
	# return
	#
	# small_lines_data=dict()
	#
	# small_lines_data[target_border_index]=disect(lines_data[target_border_index][0],other_lines)
	#
	# for k,lines in small_lines_data.items():
	# 	for line in lines:
	# 		line.plot()
	#
	# # return
	# crop_field.plot()
	#
	# plt.show()
	#
	# # for target_border_index, other_border_indexes in partitioner(0):
	# # target_lines=lines_data[target_border_index[0]]
	#
	# return
	#
	# lines=list()
	# for k,vs in lines_data.items():
	# 	for vi in vs:
	# 		lines.append([k,vi])
	# df_lines=pd.DataFrame(lines,columns=['border_index','the_line'])
	# simple_logger.debug('df_lines:\n%s',df_lines)
	# l1: TheLine=df_lines.the_line.iloc[0]
	# l2: TheLine=df_lines.the_line.iloc[1]
	# shops.split(l1.line)
	#
	# # plt.show()
	#
	# # print(xy_df)
	# # plt.scatter(xy_df.x,xy_df.y,s=0.1)
	# # plt.show()
	#
	# # xy_grid_info=np.arange(corners[0],corners[-1],1)
	#
	# # x,y=np.meshgrid()
	# # points=crop_field.get_interior_points(5,show=1)
	#
	# # print(x.shape)
	# # print(x)
	# # points=crop_field.generate_clusters(1,20)
	# # plt.scatter(points[:,0],points[:,1],c='k')
	# # plt.show()
	# return
	# cells,graph=crop_field.get_connectivity_graph(True,show='000')
	# target_cell=cells[0]
	# target_cell.plot()
	# print(target_cell)
	# # target_cell.generate_clusters(1,50,'1')
	# target_cell.get_interior_points(9,[0,1],1)
	# plt.show()
	#
	# # plt.show()
	# return
	# graph_ids=collections.defaultdict(list)
	# for cell1,cell2 in graph:
	# 	graph_ids[cell1.id].append(cell2.id)
	# for k,vs in graph_ids.items():
	# 	print(len(vs))
	# print(graph_ids)
	#
	# # crop_field.get_boustrophedon_cells(show=1)
	# # crop_field.get_field_lines(9,[0,1],1)
	#
	# plt.show()
	#
	# # return
	#
	# return
	#
	# # print(mpoly)
	#
	# # plt.imshow(img)
	# return
	# # crop_field.get_field_lines(9,show=1)
	# interior_points=crop_field.get_interior_points(1)
	# # plt.plot(points[:,0],points[:,1])
	# # target_distance=100
	# # cluster_area=target_distance**2/4*np.pi
	# # n_clusters=round(crop_field.polygon.area/cluster_area)
	# n_clusters=3
	# # simple_logger.info('n_clusters: %s',n_clusters)
	# # filename=f'kmeans_{target_distance}.joblib'
	# # if os.path.exists(filename) and False:
	# # 	kmeans=joblib.load('kmeans.joblib')
	# # else:
	# kmeans=KMeans(n_clusters=n_clusters,random_state=1)
	# kmeans.fit(interior_points)
	# # joblib.dump(kmeans,filename)
	#
	# crop_field.plot()
	# plt.scatter(kmeans.cluster_centers_[:,0],kmeans.cluster_centers_[:,1],c='k')
	# plt.show()
	#
	# # crop_field.plot()
	# # plt.show()
	# return
	# simple_logger.info('crop_field.lims:\n%s',crop_field.lims)
	# simple_logger.info('crop_field.bounds_xy:\n%s',crop_field.bounds_xy)
	# grid_size=10
	# delta=crop_field.lims[1]-crop_field.lims[0]
	# simple_logger.info('delta: %s',delta)
	# # steps=delta/grid_size
	# # simple_logger.info('steps: %s',steps)
	# # ceiled_steps=np.ceil(steps)
	# # simple_logger.info('ceiled_steps: %s',ceiled_steps)
	# # corrected_grid_size=delta/ceiled_steps
	# # simple_logger.info('corrected_grid_size: %s',corrected_grid_size)
	# pixel_count=np.ceil(delta/grid_size)[::-1].astype(int)
	# simple_logger.info('pixel_count: %s',pixel_count)
	# img_array=np.zeros(shape=pixel_count)
	# # simple_logger.info('img_array:\n%s',img_array)
	#
	# interior_points=np.round(crop_field.get_interior_points(grid_size),decimals=1)[:,[1,0]]
	# # simple_logger.info('interior_points:\n%s',interior_points)
	# positions_in_grid=np.floor_divide(interior_points,grid_size).astype(int)
	# # simple_logger.info('positions_in_grid:\n%s',positions_in_grid)
	# # positions_with_ones=np.ones((positions_in_grid.shape[0],positions_in_grid.shape[1]+1),dtype=int)
	# # positions_with_ones[:,:-1]=positions_in_grid
	# # simple_logger.info('positions_with_ones:\n%s',positions_with_ones)
	# img_array[positions_in_grid[:,0],positions_in_grid[:,1]]=1
	# print(img_array)
	# # simple_logger.info('img_array:\n%s',img_array)
	# # separate_img,cells=boustrophedon.bcd(img_array)
	# # simple_logger.info('cells: %s',cells)
	#
	# return
	# print(crop_field.all_xy)
	# # crop_field=ThePoly(shg.Polygon([[0,0],[200,-10],[90,110],[10,100]]))
	# borderlines=[TheLine(line=x,show='',width=target_distance/2) for x in
	#              crop_field.buffer(-1e-9,join_style=shg.JOIN_STYLE.mitre).get_exterior_lines(show=0)]
	# crop_field.plot()
	# # vor=spatial.Voronoi(np.random.randint(0,100,(3,2)))
	# tri=spatial.Delaunay(crop_field.all_xy)
	# plt.triplot(crop_field.all_xy[:,0],crop_field.all_xy[:,1],tri.simplices)
	# plt.plot(crop_field.all_xy[:,0],crop_field.all_xy[:,1],'o')
	# # spatial.voronoi_plot_2d(vor,ax=plt.gca())
	# plt.show()
	return

# return


# return

if __name__=='__main__':
	main()
