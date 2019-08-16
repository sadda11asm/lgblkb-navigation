import shapely.affinity as shaff
import matplotlib.pyplot as plt
import collections
import itertools

import shapely.geometry as shg
from box import Box

from egistic_navigation.base_geometry.geom_utils import min_dist
from egistic_navigation.base_geometry.line_utils import TheLine
from egistic_navigation.base_geometry.point_utils import ThePoint
from egistic_navigation.base_geometry.poly_utils import ThePoly,TheChunk
from egistic_navigation.global_support import simple_logger,with_logging
import numpy as np
from egistic_navigation import global_support as gsup

#
# class DecisionLine(TheLine):
#
# 	def __init__(self,coordinates,parent=None,**kwargs):
# 		super(DecisionLine,self).__init__(coordinates=coordinates,**kwargs)
# 		self.parents=[parent] if parent else []
#
# class FieldPoint(ThePoint):
#
# 	def __init__(self,xy,parent=None,is_border_point=True,**box_kwargs):
# 		super(FieldPoint,self).__init__(xy=xy,**box_kwargs)
# 		# parent=box_kwargs.pop('parent',None)
# 		self.parents=[parent] if parent else []
# 		self.is_border_point=is_border_point#box_kwargs.pop('is_exterior',None)
# 	pass

# class TheField(ThePoly):
#
# 	def __init__(self,polygon,parent=None,**data):
# 		super(TheField,self).__init__(polygon=polygon,**data)
# 		# self.line_type=DecisionLine
# 		# self.point_type=FieldPoint
# 		self.__graph=None
# 		self.__extension_lines=list()
# 		self.parent=parent
#
# 	def generate_graph(self,parent_graph=None):
# 		G=parent_graph or nx.Graph()
# 		# line_kwargs=dict(box_kwargs,**(line_opts or {}))
# 		lines=self.get_exterior_lines()
# 		# vertex_angles=self.get_angles_at_vertices()
# 		# if self.parent: vertex_angles=360-vertex_angles
#
# 		# point_kwargs=dict()
# 		# point_kwargs.update(box_kwargs)
# 		# point_kwargs.update(point_opts or {})
#
# 		for i in range(len(lines)):
# 			next_index=(i+1) if i+1==len(lines) else 0
#
# 			node=ThePoint(self.xy[i])
# 			next_node=ThePoint(self.xy[next_index])
#
# 			G.add_node(node,lines=[lines[i],lines[i-1]])
# 			G.add_edge(node,next_node,line=lines[i])
#
# 		for hole in self.holes:
# 			hole.generate_graph(G)
#
# 		# for i, xy in enumerate(self.xy):
# 		# 	G.add_node(i,)
#
# 		# for i,line in enumerate(lines):
# 		# 	# G.add_edge(*map(ThePoint,line.xy),line=line)
# 		# 	if i+1==len(lines):
# 		# 		G.add_edge(i,0,line=line)
# 		# 	else:
# 		# 		G.add_edge(i,i+1,line=line)
# 		return G
#
# 	@property
# 	def G(self):
# 		if self.__graph is None: self.__graph=self.generate_graph()
# 		return self.__graph
#
# 	@with_logging()
# 	def get_extension_lines(self,parent=None,show=False):
# 		the_main_poly=parent or self  #ThePoly(self.p)#self
# 		if self.is_convex:
# 			simple_logger.info('The field is convex. Extension lines are absent.')
# 			return self
#
# 		delta=self.convex_hull.p.difference(self.p)
# 		if delta.is_empty:
# 			simple_logger.info('Delta is empty. Skipping.')
# 			return self
# 		elif isinstance(delta,shg.MultiPolygon):
# 			chunks=[TheChunk(x,self,id=f'Chunk-{i}') for i,x in enumerate(delta)]
# 		elif isinstance(delta,shg.Polygon):
# 			chunks=[TheChunk(delta,self)]
# 		else:
# 			simple_logger.critical('delta: %s',delta)
# 			# self.plot()
# 			# self.convex_hull.plot(c='b')
# 			# plt.show()
# 			raise NotImplementedError
#
# 		out_extension_lines=list()
# 		for i_chunk,the_chunk in enumerate(chunks):
# 			# if i_chunk!=3: continue
# 			chunk_hull=the_chunk.convex_hull  #.plot(c='b',lw=0.5,ls='--',text=f'chunk_hull-{i_chunk}')
# 			for i_point,xy in enumerate(the_chunk.xy[:-1]):
# 				xy_point=ThePoint(xy,)  #.plot(f'{i_point}')
# 				# simple_logger.debug('Point-%s',i_point)
# 				# the_chunk.baseline.plot(c='r')
# 				if not the_chunk.touches_parent or (chunk_hull.touches(xy) and the_chunk.baseline.line.distance(xy_point.point)>min_dist):
# 					# simple_logger.info('Point-%s',i_point)
# 					# extension_lines=the_chunk[xy]['extension_lines']=the_main_poly[xy]['extension_lines']=list()
# 					extension_lines=the_main_poly[xy]['extension_lines']=list()
# 					for i_line,line in enumerate(the_chunk[xy]['lines']):
# 						line: TheLine=line  #.plot(text=str(i_line))
# 						extension_line=line.extend_from(xy,the_main_poly.cover_line_length)
# 						cropped_line=the_main_poly.p.intersection(extension_line.line)
# 						# simple_logger.debug('cropped_line:\n%s',cropped_line)
# 						if isinstance(cropped_line,shg.MultiLineString):
# 							cropped_line=cropped_line[0]
# 						elif isinstance(cropped_line,shg.GeometryCollection):
# 							continue
#
# 						# self.plot()
# 						cropped_extension_line=TheLine(cropped_line)  #.plot()
# 						# xy_point.plot(i_point)
# 						# cropped_extension_line.plot()
# 						# plt.show()
# 						if xy_point in cropped_extension_line:
# 							if cropped_extension_line.line.length<1e-3:
# 								simple_logger.warning('Too short extension line encountered.')
# 							extension_lines.append(cropped_extension_line)
# 							out_extension_lines.append(cropped_extension_line)
#
# 			geom_collection=chunk_hull.p.intersection(self.p)
# 			sub_chunks=list()
# 			for geom_obj in geom_collection:
# 				if isinstance(geom_obj,shg.LineString):
# 					# TheLine(geom_obj).plot(text='line',c='y')
# 					pass
# 				elif isinstance(geom_obj,shg.MultiLineString):
# 					# [TheLine(x).plot(text='line',c='cyan') for x in geom_obj]
# 					pass
# 				elif isinstance(geom_obj,shg.Polygon):
# 					sub_chunk=TheField(geom_obj)  #.plot(text='ThePoly',c='r')
# 					sub_chunks.append(sub_chunk)
# 				else:
# 					simple_logger.debug('geom_obj: %s',geom_obj)
# 					raise NotImplementedError
#
# 			for smaller_chunk in sub_chunks:
# 				if not smaller_chunk.is_convex:
# 					# the_chunk.plot(f'the_chunk-{i_chunk}')
# 					# smaller_chunk.plot('smaller_chunk',c='r',lw=0.3)
# 					# smaller_chunk.id="_".join(map(str,[self.id,'smaller_chunk']))
# 					# simple_logger.debug('smaller_chunk:\n%s',smaller_chunk)
# 					smaller_chunk.get_extension_lines(parent=the_main_poly)
#
# 		if show:
# 			for ext_line in out_extension_lines:
# 				ext_line.plot()
# 		simple_logger.debug('Extension lines count: %s',len(out_extension_lines))
# 		return out_extension_lines
#
# 	@property
# 	def extension_lines(self):
# 		if not self.__extension_lines:
# 			self.__extension_lines=self.get_extension_lines()
# 		return self.__extension_lines
#
# 	def get_angles_at_vertices(self):
# 		vectors=self.xy[1:]-self.xy[:-1]
# 		line_angles=np.arctan2(vectors[:,1],vectors[:,0])*180/np.pi
# 		line_angles=np.append(line_angles,line_angles[0])
# 		angle_diff=((line_angles[1:]-line_angles[:-1])+180)%360
# 		angle_at_vertices=np.zeros(angle_diff.shape)
# 		angle_at_vertices[0]=angle_diff[-1]
# 		angle_at_vertices[1:]=angle_diff[:-1]
# 		return angle_at_vertices
#
# 	def __getitem__(self,item):
# 		# simple_logger.debug('item: %s',item)
# 		if type(item) in (np.ndarray,tuple,list):
# 			# if isinstance(item,np.ndarray): item=item.ravel()
# 			u,v=item
# 			return self.G.nodes[u,v]
# 		elif isinstance(item,int):
# 			simple_logger.warning('Oppa!!!')
# 			return self.exterior_lines[item]
# 		else:
# 			return super().__getitem__(item)


class FieldPoly(ThePoly):
	
	def __init__(self,polygon,**data):
		super(FieldPoly,self).__init__(polygon=polygon,**data)
		self._decision_points=collections.OrderedDict()
		self._adjacency_info=dict()
	
	# def _add_node(self,G,xy,parent_lines):
	# 	node=ThePoint(xy)
	# 	G.add_node(node,lines=parent_lines)
	# 	return node
	
	@with_logging()
	def get_extension_lines(self,parent=None,show=False):
		the_main_poly=parent or self  #ThePoly(self.p)#self
		if self.is_convex and not self.is_multilinestring:
			simple_logger.info('The field is convex. Extension lines are absent.')
			return []
		
		delta=self.convex_hull.p.difference(self.p)
		if delta.is_empty:
			simple_logger.info('Delta is empty. Skipping.')
			return []
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
			# if i_chunk!=2: continue
			# the_chunk.plot(text=i_chunk)
			convex_chunk=the_chunk.convex_hull  #.plot(c='b',lw=0.5,ls='--',text=f'convex_chunk-{i_chunk}')
			for i_point,xy in enumerate(the_chunk.xy[:-1]):
				xy_point=ThePoint(xy,)  #.plot(f'{i_point}')
				# simple_logger.debug('Point-%s',i_point)
				# the_chunk.baseline.plot(c='r')
				# if i_point==5:
				# 	simple_logger.warning('i_point: %s',i_point)
				# 	simple_logger.debug('convex_chunk.touches(xy): %s',convex_chunk.touches(xy))
				if the_chunk.touches_parent and (not convex_chunk.touches(xy) or xy_point in the_chunk.baseline): continue
				
				# if not the_chunk.touches_parent or (convex_chunk.touches(xy) and not xy_point in the_chunk.baseline):
				# simple_logger.info('Point-%s',i_point)
				# extension_lines=the_chunk[xy]['extension_lines']=the_main_poly[xy]['extension_lines']=list()
				extension_lines=the_main_poly[xy]['extension_lines']=list()
				for chunk_line in the_chunk[xy]['lines']:
					# self.plot()
					# line: TheLine=chunk_line.plot()
					# line[0].plot('Start')
					# line[1].plot('End')
					extension_line=chunk_line.extend_from(xy,the_main_poly.cover_line_length)  #.plot(text='extension_line')
					# plt.show()
					
					cropped_line=the_main_poly.p.intersection(extension_line.line)
					# simple_logger.debug('cropped_line:\n%s',cropped_line)
					if isinstance(cropped_line,shg.MultiLineString):
						res_lines=[TheLine(x) for x in cropped_line]
						for res_line in res_lines:
							if xy_point in res_line:
								cropped_line=res_line
								break
					elif isinstance(cropped_line,shg.GeometryCollection):
						continue
					
					# self.plot()
					cropped_extension_line=TheLine(cropped_line)  #.plot()
					# xy_point.plot(i_point)
					# cropped_extension_line.plot()
					# plt.show()
					if not xy_point in cropped_extension_line: continue
					
					if cropped_extension_line.line.length<1e-3:
						simple_logger.warning('Too short extension line encountered.')
					
					extension_lines.append(cropped_extension_line)
					if cropped_extension_line in out_extension_lines:
						simple_logger.warning('Duplicate extension line detected. Skipping.')
						continue
					out_extension_lines.append(cropped_extension_line)
			
			geom_collection=convex_chunk.p.intersection(self.p)
			smaller_chunks=list()
			for geom_obj in geom_collection:
				if isinstance(geom_obj,shg.LineString):
					# TheLine(geom_obj).plot(text='line',c='y')
					pass
				elif isinstance(geom_obj,shg.MultiLineString):
					# [TheLine(x).plot(text='line',c='cyan') for x in geom_obj]
					pass
				elif isinstance(geom_obj,shg.Polygon):
					if geom_obj.area<min_dist: continue
					smaller_chunk=ThePoly(geom_obj)  #.plot(text='ThePoly',c='r')
					smaller_chunks.append(smaller_chunk)
				else:
					simple_logger.debug('geom_obj: %s',geom_obj)
					raise NotImplementedError
			
			for i_smaller_chunk,smaller_chunk in enumerate(smaller_chunks):
				# smaller_chunk.plot(text=f"{i_smaller_chunk}")
				if not smaller_chunk.is_convex:
					# the_chunk.plot(f'the_chunk-{i_chunk}')
					# smaller_chunk.plot('smaller_chunk',c='r',lw=0.3)
					# smaller_chunk.id="_".join(map(str,[self.id,'smaller_chunk']))
					# simple_logger.debug('smaller_chunk:\n%s',smaller_chunk)
					smaller_chunk_extension_lines=FieldPoly(smaller_chunk).get_extension_lines(parent=the_main_poly)
					out_extension_lines.extend(smaller_chunk_extension_lines)
		
		extension_points={x[1]:x for x in out_extension_lines}
		for extension_point in extension_points:
			for line in self.all_lines:
				# self.plot()
				# line.plot(text='LINE')
				# extension_point.plot('EP')
				# plt.show()
				# if line.line.distance(extension_point.point)<min_dist:
				if line.almost_touches(extension_point):
					self._add_node(self.G,extension_point,
					               parent_lines=[extension_points[extension_point],line],is_intermediate=True)
		
		self.generate_intersection_points(out_extension_lines)
		
		if show:
			for ext_line in out_extension_lines:
				ext_line.plot()
		simple_logger.debug('Extension lines count: %s',len(out_extension_lines))
		return out_extension_lines
	
	@property
	def extension_lines(self):
		if not self._extension_lines:
			self._extension_lines=self.get_extension_lines()
		return self._extension_lines
	
	def add_angle_info(self,parent=None):
		parent=parent or self
		for i,exterior_point in enumerate(self.points):
			parent.G.nodes[exterior_point]['angle']=self.angles_at_vertices[i]
		for hole in self.holes:
			hole.add_angle_info(parent=parent)
		return self
	
	def generate_decision_points(self,show=False):
		simple_logger.debug('all_points count: %s',len(self.all_points))
		extension_points={x[1]:x for x in self.extension_lines}
		simple_logger.debug('extension_points count: %s',len(extension_points))
		
		border_points=[*self.all_points,*extension_points]
		simple_logger.info('border_points count: %s',len(border_points))
		
		inner_points=self.generate_intersection_points(self.extension_lines)
		
		# decision_points=collections.OrderedDict()
		decision_points=Box(ordered_box=True)
		decision_points['border']=border_points
		decision_points['inner']=inner_points
		decision_points['all']=[*border_points,*inner_points]
		
		if len(decision_points['all'])!=len(set(decision_points['all'])):
			simple_logger.warning('Mergeable points detected.')
		
		# decision_points=set(decision_points)
		simple_logger.info('decision_points count: %s',len(decision_points['all']))
		self.G.decision_points=self.data.decision_points=decision_points
		# self.data.decision_points=decision_points
		if show:
			for decision_point in decision_points['all']:
				decision_point.plot()
		return decision_points
	
	@property
	def decision_points(self):
		if not self._decision_points:
			self._decision_points=self.generate_decision_points()
		return self._decision_points
	
	def generate_adjacency_info(self):
		decision_points=self.decision_points
		for line in [*self.all_lines,*self.extension_lines]:
			# if line.intermediate_points:
			# 	line.intermediate_points=sorted(line.intermediate_points,key=lambda ip:line[0].point.distance(ip.point))
			# self.plot()
			# line.plot()
			all_line_points=line.get_all_points()
			for i,line_point in enumerate(all_line_points[:-1]):
				sub_line=TheLine([line_point,all_line_points[i+1]])  #.plot(text=f'{i}')
				
				self.G.add_edge(line_point,all_line_points[i+1],
				                line=sub_line,
				                parent_line=line,
				                )
				# line_point.plot(f'LP-{i}')
				pass
		return {n:set(node_data) for n,node_data in self.G.adjacency()}
	
	def get_altitude(self,show=False):
		crop_field=self.__class__(self.geometry)
		for line in crop_field.all_lines: crop_field.G.add_edge(line[0],line[1],line=line)
		adjacency_data={n:list(neighbors.keys()) for n,neighbors in crop_field.G.adjacency()}
		altitude=0
		factor=1
		# sorted_points=sorted(mi.flatten(map(lambda h:h.points,crop_field.holes)),key=lambda p:p.x)
		sorted_points=sorted(crop_field.all_points,key=lambda p:p.x)
		for i_point,point in enumerate(sorted_points[1:],start=1):
			# if point.x==sorted_points[i_point-1].x: simple_logger.warning('Points have same x vallues!!!. Wrong altitude!')
			
			if show: point.plot(f'P-{i_point}')
			# simple_logger.debug('factor: %s',factor)
			altitude+=factor*(point.x-sorted_points[i_point-1].x)
			neighbors=adjacency_data[point]
			if all(map(lambda n:n.x>point.x,neighbors)):
				factor+=1
			elif all(map(lambda n:n.x<point.x,neighbors)):
				factor-=1
		
		if factor!=0:
			simple_logger.warning('Final factor value is not 0, but %s',factor)
		# simple_logger.debug('altitude: %s',altitude)
		return altitude
	
	def get_min_altitude(self,use_mp=False):
		if use_mp:
			alt_getter=lambda _exterior_line:(
			FieldPoly(shaff.rotate(self.geometry,180-_exterior_line.angle,_exterior_line[0].geometry)).get_altitude(),_exterior_line)
			altitude_data=gsup.ParallelTasker(alt_getter).set_run_params(_exterior_line=self.all_lines).run(sleep_time=1e-4)
			min_altitude,base_line=sorted(altitude_data,key=lambda d:d[0])[0]
		else:
			base_line=None
			min_altitude=np.inf
			for exterior_line in self.all_lines:
				cost=FieldPoly(shaff.rotate(self.geometry,180-exterior_line.angle,exterior_line[0].geometry)).get_altitude()
				if cost<min_altitude:
					min_altitude=cost
					base_line=exterior_line
			assert base_line is not None
			# crop_field.plot()
			# min_costing_rotated_field.plot()
			# exterior_line.plot(c='r')
			# exterior_line[0].plot('P')
			# simple_logger.debug('exterior_line.angle: %s',exterior_line.angle)
			# plt.show()
		simple_logger.a().debug('min_altitude: %s',min_altitude).s()
		self.data.base_line=base_line
		return min_altitude,base_line
		pass
	
	@property
	def adjacency_info(self):
		if not self._adjacency_info:
			self._adjacency_info=self.generate_adjacency_info()
		return self._adjacency_info
	
	# def get_optimal_field_lines(self,field_line_offset,with_baseline=False,show=False):
	# 	# self.plot()
	# 	# plt.show()
	#
	# 	field_lines_data=self.get_field_lines(field_line_offset,show=show)
	# 	if not field_lines_data:
	# 		if with_baseline: return None,None
	# 		else: return None
	# 	# for k,lines in field_lines_data.items():
	# 	# 	if isinstance(k,tuple)
	# 	sorted_field_lines=sorted(field_lines_data.values(),key=lambda x:len(x))
	# 	field_lines=sorted_field_lines[0]
	# 	# for field_line in field_lines:
	# 	# 	field_line.plot()
	#
	# 	# border_index=None
	# 	baseline=None
	#
	# 	if with_baseline:
	# 		# for p in non_collinear_field.points:
	# 		# 	p.plot()
	# 		# plt.show()
	# 		non_collinear_field=self.as_noncollinear().plot()
	# 		for i_exterior_line,exterior_line in enumerate(non_collinear_field.exterior_lines):
	# 			exterior_line.plot()
	# 			# simple_logger.debug('exterior_line.angle:\n%s',exterior_line.angle)
	# 			# simple_logger.debug('field_lines[0].angle:\n%s',field_lines[0].angle)
	# 			angle_diff=abs(exterior_line.angle-field_lines[0].angle)
	# 			# simple_logger.debug('angle_diff: %s',angle_diff)
	# 			if angle_diff<min_dist or abs(180-angle_diff)<min_dist:
	# 				# simple_logger.debug('angle_diff: %s',angle_diff)
	# 				border_index=i_exterior_line
	# 				baseline=non_collinear_field.exterior_lines[border_index]
	# 				_lines=baseline.get_field_lines(self)
	# 				if _lines: baseline=_lines[0]
	#
	# 				break
	# 		# plt.show()
	#
	# 		if with_baseline: assert baseline is not None
	# 	# fl_counters=[len(x) for x in sorted_field_lines]
	# 	# simple_logger.debug('# of field lines along borders: %s',fl_counters)
	#
	# 	if show:
	# 		for i,field_line in enumerate(field_lines):
	# 			# field_line.plot(text=f'{i}')
	# 			field_line.plot()
	# 		self.plot(text=f'{len(field_lines)}')
	# 	# field_line[0].plot('S')
	# 	# field_line[1].plot('F')
	#
	# 	if with_baseline: return field_lines,baseline
	# 	else: return field_lines
	
	def get_field_lines(self,offset_distance,base_line,show=False):
		out_field_lines=list()
		shrinked_poly=self.geometry.buffer(-offset_distance/2,join_style=shg.JOIN_STYLE.mitre)
		if shrinked_poly.is_empty:
			pass
		elif isinstance(shrinked_poly,shg.MultiPolygon):
			for p in shrinked_poly:
				sub_field=FieldPoly(p).plot()
			# sub_field_lines_data=sub_field.get_field_lines(offset_distance,show=True)
			plt.show()
			raise NotImplementedError
		
		# 	for k,lines in sub_field_lines_data.items():
		# 		field_lines_data[sub_field.id,k].extend(lines)
		
		else:
			shrinked_field=FieldPoly(shrinked_poly)
			lines_count_to_cover=int(np.ceil(self.cover_line_length/offset_distance))
			
			# shrinked_field.plot()
			def parent_field_lines_getter(parent_line):
				_child_field_lines=list()
				offset_lines=parent_line.get_field_lines(self)
				
				# offset_lines=list()
				
				offset_lines.extend(parent_line.offset_by(offset_distance,lines_count_to_cover))
				offset_lines.extend(parent_line.offset_by(-offset_distance,lines_count_to_cover))
				
				# for i in range(len(offset_lines)): offset_lines[i]=offset_lines[i].reversed()
				for offset_line in offset_lines:
					field_lines=offset_line.get_field_lines(self)
					for field_line in field_lines:
						if not field_line: continue
						if show: field_line.plot()
						_child_field_lines.append(field_line)
				return _child_field_lines
			
			# if field_line.geometry.within(self.geometry):
			# if use_mp:
			# 	child_field_lines_data=gsup.ParallelTasker(parent_field_lines_getter)\
			# 		.set_run_params(parent_line=shrinked_field.all_lines).run(sleep_time=1e-3)
			#
			# 	for i_parent_line,child_field_lines in enumerate(child_field_lines_data):
			# 		# parent_line: TheLine=parent_line
			# 		field_lines_data[i_parent_line]=child_field_lines
			# else:
			counter=0
			for i_parent_line,_parent_line in enumerate(shrinked_field.all_lines):
				if base_line is None or abs((_parent_line.angle+180)%180-(base_line.angle+180)%180)<min_dist:
					out_field_lines=parent_field_lines_getter(_parent_line)
					# if not base_line is None: break
					counter+=1
			if counter!=1:
				simple_logger.warning('counter: %s',counter)
			
			# else:
			# 	if abs((_parent_line.angle+180)%180-(base_line.angle+180)%180)<min_dist:
			# 		field_lines_data[i_parent_line]=parent_field_lines_getter(_parent_line)
			
			# else:
			# 	for i_parent_line,_parent_line in enumerate(shrinked_field.all_lines):
			# 		if abs((_parent_line.angle+180)%180-(base_line.angle+180)%180)<min_dist:
			# 			field_lines_data[i_parent_line]=parent_field_lines_getter(_parent_line)
		
		return out_field_lines
	
	# field_lines_data[i].extend(field_lines_from_single_line)
	
	def get_subfields(self,start_node,offset_distance,show=False):
		#adjacency_data={n:set(node_data) for n,node_data in self.G.adjacency()}
		# simple_logger.debug('len(adjacency_data): %s',len(adjacency_data))
		
		self.add_angle_info()
		previous_subfields=list()
		for path in dfs_paths(self,self.adjacency_info,start_node):
			outputs=list()
			subfield=FieldPoly(path)
			# self.plot()
			# subfield.plot()
			# plt.show()
			# continue
			is_seen=False
			for previous_subfield in previous_subfields:
				if subfield==previous_subfield:
					is_seen=True
					# simple_logger.info('is_seen: %s',is_seen)
					break
			if is_seen: break
			previous_subfields.append(subfield)
			outputs.append(subfield)
			try:
				diff_result=self.geometry.difference(subfield.geometry)
			except:
				continue
			if diff_result.is_empty:
				yield outputs
				continue
			elif isinstance(diff_result,shg.MultiPolygon):
				remaining_polys=list()
				for diff_chunk in diff_result:
					if diff_chunk.area<min_dist: continue
					remaining_field=FieldPoly.as_valid(diff_chunk)  #.plot(c='g')
					remaining_polys.append(remaining_field)
				if len(remaining_polys)>1:
					# for remaining_poly in remaining_polys:
					# 	remaining_poly.plot(lw=5,alpha=0.3)
					continue
				remaining_field=remaining_polys[0]
			else:
				# simple_logger.debug('diff_result:\n%s',diff_result)
				remaining_field=FieldPoly.as_valid(diff_result)  #.plot(c='g')
			
			for hole in self.holes:
				if hole.geometry.representative_point().within(remaining_field.geometry):
					try:
						remaining_poly=remaining_field.geometry.difference(hole.geometry)
					except:
						remaining_poly=shg.Polygon()
					remaining_field=FieldPoly(remaining_poly)
			
			shrinked_resultant_poly=remaining_field.geometry.buffer(-offset_distance/2,join_style=shg.JOIN_STYLE.mitre)
			
			if shrinked_resultant_poly.is_empty:
				outputs.append(shrinked_resultant_poly)
			elif isinstance(shrinked_resultant_poly,shg.MultiPolygon):
				for shrinked_poly in shrinked_resultant_poly:
					expanded_poly=shrinked_poly.buffer(offset_distance/2,join_style=shg.JOIN_STYLE.mitre)
					expanded_field=FieldPoly(expanded_poly)
					outputs.append(expanded_field)
			else:
				expanded_poly=shrinked_resultant_poly.buffer(offset_distance/2,join_style=shg.JOIN_STYLE.mitre)
				expanded_field=FieldPoly(expanded_poly)
				outputs.append(expanded_field)
			
			if show:
				subfield.plot(lw=5)
				if not remaining_field.is_empty:
					remaining_field.plot(lw=5,alpha=0.3)
				start_node.plot('Start')
			# for extension_line in self.extension_lines:
			# 	extension_line.plot(alpha=0.5)
			# subfield.get_optimal_field_lines(24,show=True)
			
			yield outputs
			# for line in raw_poly.exterior_lines:
			# 	simple_logger.debug('line.angle: %s',line.angle)
			# for l1,l2 in itertools.combinations(raw_poly.exterior_lines,2):
			#
			# 	result=l1.geometry.distance(l2.geometry)==0 and abs(l1.angle-l2.angle)<min_dist
			# 	if result:
			# 		simple_logger.info('l1.angle: %s',l1.angle)
			# 		self.plot()
			# 		raw_poly.plot(lw=5,c='r')
			# 		l1.plot(text='L1')
			# 		l2.plot(text='L2')
			# 		plt.show()
			
			# for i,field_lines in field_lines_data.items():
			
			# start_node.plot('Start')
			# goal.plot('Goal')
			# for i,node in enumerate(path):
			# 	if i in [0,len(path)-1]: continue
			# 	angle=self.G.nodes[node].get('angle')
			# 	if angle:
			# 		node.plot(f"""Node-{i}, {angle:.2f}""")
			# 	else:
			# 		node.plot(f'Node-{i}')
			# plt.show()
			pass
	
	def play_with_sub_field(self,p1,field_lines,start,):
		# simple_logger.warning('border_index:\n%s',border_index)
		field_lines=sorted(field_lines,key=lambda _line:_line.geometry.distance(start.geometry))
		first_field_line=TheLine(sorted(field_lines[0][:],key=lambda p:p.geometry.distance(start.geometry)))
		# simple_logger.debug('first_field_line.angle: %s',first_field_line.angle)
		
		start_side_points=list()
		
		for i,field_line in enumerate(field_lines):
			# simple_logger.debug('field_line.angle: %s',field_line.angle)
			if abs(field_line.angle-first_field_line.angle)<min_dist:
				start_side_points.append(field_line[0])
			else:
				start_side_points.append(field_line[1])
			field_line.plot()
		start_side_lines=set()
		for i,start_side_point in enumerate(start_side_points):
			# start_side_point.plot(text=f'S-{i}')
			for collinear_exterior_line in p1.as_noncollinear().exterior_lines:
				if collinear_exterior_line.almost_touches(start_side_point):
					# collinear_exterior_line.plot(c='k')
					for actual_exterior_line in p1.exterior_lines:
						if actual_exterior_line[0].almost_touches(collinear_exterior_line) and\
								actual_exterior_line[1].almost_touches(collinear_exterior_line):
							start_side_lines.add(actual_exterior_line)
							actual_exterior_line.plot(c='k')
		
		for point in p1.points:
			point.plot(c='k')  # simple_logger.warning('border_index:\n%s',border_index)
		field_lines=sorted(field_lines,key=lambda _line:_line.geometry.distance(start.geometry))
		first_field_line=TheLine(sorted(field_lines[0][:],key=lambda p:p.geometry.distance(start.geometry)))
		# simple_logger.debug('first_field_line.angle: %s',first_field_line.angle)
		
		start_side_points=list()
		
		for i,field_line in enumerate(field_lines):
			# simple_logger.debug('field_line.angle: %s',field_line.angle)
			if abs(field_line.angle-first_field_line.angle)<min_dist:
				start_side_points.append(field_line[0])
			else:
				start_side_points.append(field_line[1])
			field_line.plot()
		start_side_lines=set()
		for i,start_side_point in enumerate(start_side_points):
			# start_side_point.plot(text=f'S-{i}')
			for collinear_exterior_line in p1.as_noncollinear().exterior_lines:
				if collinear_exterior_line.almost_touches(start_side_point):
					# collinear_exterior_line.plot(c='k')
					for actual_exterior_line in p1.exterior_lines:
						if actual_exterior_line[0].almost_touches(collinear_exterior_line) and\
								actual_exterior_line[1].almost_touches(collinear_exterior_line):
							start_side_lines.add(actual_exterior_line)
							actual_exterior_line.plot(c='k')
		
		for point in p1.points:
			point.plot(c='k')

# def get_dfs_paths(graph,_start,goal):
# 	stack=[(_start,[_start])]
# 	while_counter=itertools.count()
# 	while stack:
# 		next_while_counter=next(while_counter)
# 		# simple_logger.debug('next_while_counter: %s',next_while_counter)
#
# 		(vertex,path)=stack.pop()
# 		for_counter=itertools.count()
# 		for next_node in graph[vertex]-set(path):
# 			next_for_counter=next(for_counter)
# 			# simple_logger.debug('next_for_counter: %s',next_for_counter)
# 			the_path=path+[next_node]
# 			path_length=len(the_path)
# 			if path_length>2:
# 				the_poly=ThePoly(the_path)
# 				# the_poly.plot()
# 				# plt.show()
# 				# simple_logger.debug('the_poly.geometry.area: %s',the_poly.geometry.area)
# 				if not the_poly.is_convex:
# 					continue
# 				if next_node==goal:
# 					if the_poly.geometry.area>min_dist:
# 						yield the_path
# 				else:
# 					stack.append((next_node,the_path))
# 			else:
# 				stack.append((next_node,the_path))

def dfs_paths(field_poly: FieldPoly,graph,start,):
	hole_points=[hole.geometry.representative_point() for hole in field_poly.holes]
	
	def get_dfs_paths(_start,goal):
		stack=[(_start,[_start])]
		while_counter=itertools.count()
		while stack:
			next_while_counter=next(while_counter)
			# simple_logger.debug('next_while_counter: %s',next_while_counter)
			
			(vertex,path)=stack.pop()
			for_counter=itertools.count()
			for next_node in graph[vertex]-set(path):
				next_for_counter=next(for_counter)
				# simple_logger.debug('next_for_counter: %s',next_for_counter)
				the_path=path+[next_node]
				path_length=len(the_path)
				if path_length>2:
					the_poly=ThePoly(the_path)
					
					# if the_poly.geometry.area>min_dist:
					contains_hole=False
					for point_in_hole in hole_points:
						if point_in_hole.within(the_poly.geometry):
							contains_hole=True
							break
					# the_poly.plot()
					# plt.show()
					# simple_logger.debug('the_poly.geometry.area: %s',the_poly.geometry.area)
					# simple_logger.debug('contains_hole: %s',contains_hole)
					# simple_logger.debug('the_poly.is_convex: %s',the_poly.is_convex)
					if contains_hole or (the_poly.geometry.area>min_dist and not the_poly.is_convex):
						continue
					# field_poly.plot()
					# the_poly.plot()
					# plt.show()
					# simple_logger.debug('next_node:\n%s',next_node)
					# simple_logger.debug('goal:\n%s',goal)
					# simple_logger.warning('path_length: %s',path_length)
					if next_node==goal:
						if the_poly.geometry.area>min_dist:
							yield the_path
					else:
						stack.append((next_node,the_path))
				else:
					stack.append((next_node,the_path))
	
	for start_neighbor in graph[start]:
		for path_from_neighbor in get_dfs_paths(start_neighbor,start):
			closed_path=[start]+path_from_neighbor
			yield closed_path

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
	# cells,G=crop_field.get_connectivity_graph(True,show='000')
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
	# for cell1,cell2 in G:
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
