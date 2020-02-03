import collections
import itertools

import matplotlib.pyplot as plt
import more_itertools as mi
import numpy as np
import shapely.affinity as shaff
import shapely.geometry as shg
# from wrapt_timeout_decorator import timeout
import visilibity as vis
from box import Box
from lgblkb_tools import logger
from lgblkb_tools.common.utils import ParallelTasker

from lgblkb_navigation.base_geometry.geom_utils import min_dist
from lgblkb_navigation.base_geometry.line_utils import TheLine
from lgblkb_navigation.base_geometry.point_utils import ThePoint
from lgblkb_navigation.base_geometry.poly_utils import ThePoly,TheChunk

path_width=12
dfs_threshold=45
search_loop_limit=3e5
workers_count=None
logger=logger
epsilon=1e-7

MinCoster=collections.namedtuple('MinCoster',['cost','base_line','width_height_ratio'])

class FieldPoly(ThePoly):
	
	def __init__(self,polygon,**data):
		super(FieldPoly,self).__init__(polygon=polygon,**data)
		self._decision_points=collections.OrderedDict()
		self._adjacency_info=dict()
	
	# self.vis_poly=
	
	# def _add_node(self,G,xy,parent_lines):
	# 	node=ThePoint(xy)
	# 	G.add_node(node,lines=parent_lines)
	# 	return node
	
	@logger.trace()
	def get_extension_lines(self,parent=None,show=False):
		the_main_poly=parent or self  #ThePoly(self.p)#self
		if self.is_convex and not self.is_multilinestring:
			logger.info('The field is convex. Extension lines are absent.')
			return []
		
		delta=self.convex_hull.p.difference(self.p)
		if delta.is_empty:
			logger.info('Delta is empty. Skipping.')
			return []
		elif isinstance(delta,shg.MultiPolygon):
			chunks=[TheChunk(x,self,id=f'Chunk-{i}') for i,x in enumerate(delta)]
		elif isinstance(delta,shg.Polygon):
			chunks=[TheChunk(delta,self)]
		else:
			logger.critical('delta: %s',delta)
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
				# logger.debug('Point-%s',i_point)
				# the_chunk.baseline.plot(c='r')
				# if i_point==5:
				# 	logger.warning('i_point: %s',i_point)
				# 	logger.debug('convex_chunk.touches(xy): %s',convex_chunk.touches(xy))
				if the_chunk.touches_parent and (not convex_chunk.touches(xy) or xy_point in the_chunk.baseline): continue
				
				# if not the_chunk.touches_parent or (convex_chunk.touches(xy) and not xy_point in the_chunk.baseline):
				# logger.info('Point-%s',i_point)
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
					# logger.debug('cropped_line:\n%s',cropped_line)
					if isinstance(cropped_line,shg.LineString):
						pass
					elif isinstance(cropped_line,shg.MultiLineString):
						res_lines=[TheLine(x) for x in cropped_line]
						for res_line in res_lines:
							if xy_point in res_line:
								cropped_line=res_line
								break
					elif isinstance(cropped_line,shg.Point):
						continue
					elif isinstance(cropped_line,shg.GeometryCollection):
						# logger.error('cropped_line: %s',cropped_line)
						# self.plot()
						# TheLine(cropped_line[1]).plot()
						# plt.show()
						# raise NotImplementedError
						continue
					else:
						logger.error('cropped_line: %s',cropped_line)
						raise NotImplementedError
					
					# self.plot()
					try:
						cropped_extension_line=TheLine(cropped_line)  #.plot()
					except AssertionError:
						logger.debug('cropped_line: %s',cropped_line)
						raise
					# xy_point.plot(i_point)
					# cropped_extension_line.plot()
					# plt.show()
					if not xy_point in cropped_extension_line: continue
					
					if cropped_extension_line.line.length<1e-3:
						logger.warning('Too short extension line encountered.')
					
					extension_lines.append(cropped_extension_line)
					if cropped_extension_line in out_extension_lines:
						logger.warning('Duplicate extension line detected. Skipping.')
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
					logger.debug('geom_obj: %s',geom_obj)
					raise NotImplementedError
			
			for i_smaller_chunk,smaller_chunk in enumerate(smaller_chunks):
				# smaller_chunk.plot(text=f"{i_smaller_chunk}")
				if not smaller_chunk.is_convex:
					# the_chunk.plot(f'the_chunk-{i_chunk}')
					# smaller_chunk.plot('smaller_chunk',c='r',lw=0.3)
					# smaller_chunk.id="_".join(map(str,[self.id,'smaller_chunk']))
					# logger.debug('smaller_chunk:\n%s',smaller_chunk)
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
		logger.debug('Extension lines count: %s',len(out_extension_lines))
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
		logger.debug('all_points count: %s',len(self.all_points))
		extension_points={x[1]:x for x in self.extension_lines}
		logger.debug('extension_points count: %s',len(extension_points))
		
		border_points=[*self.all_points,*extension_points]
		logger.info('border_points count: %s',len(border_points))
		
		# inner_points=self.generate_intersection_points(self.extension_lines)
		
		# decision_points=collections.OrderedDict()
		decision_points=Box(ordered_box=True)
		decision_points['border']=border_points
		# decision_points['inner']=inner_points
		# decision_points['all']=[*border_points,*inner_points]
		decision_points['all']=border_points
		
		if len(decision_points['all'])!=len(set(decision_points['all'])):
			logger.warning('Mergeable points detected.')
		
		# decision_points=set(decision_points)
		logger.info('decision_points count: %s',len(decision_points['all']))
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
	
	def generate_adjacency_info_old(self):
		decision_points=self.decision_points
		self.plot()
		# for line in [*self.all_lines,*self.extension_lines]:
		for line in [*self.all_lines,*self.extension_lines]:
			# if line.intermediate_points:
			# 	line.intermediate_points=sorted(line.intermediate_points,key=lambda ip:line[0].point.distance(ip.point))
			
			line.plot()
			all_line_points=line.get_all_points()
			for i,line_point in enumerate(all_line_points[:-1]):
				sub_line=TheLine([line_point,all_line_points[i+1]])  #.plot(text=f'{i}')
				
				self.G.add_edge(line_point,all_line_points[i+1],
				                line=sub_line,
				                parent_line=line,
				                )
				line_point.plot(f'LP-{i}')
				pass
		plt.show()
		return {n:set(node_data) for n,node_data in self.G.adjacency()}
	
	def generate_adjacency_info(self):
		decision_points=self.decision_points
		# self.plot()
		# for line in [*self.all_lines,*self.extension_lines]:
		cases=list()
		cases.append([self.all_lines,lambda _line:_line.get_all_points()])
		cases.append([self.extension_lines,lambda _line:_line[:]])
		
		for bunch_lines,point_getter in cases:
			for line in bunch_lines:
				# if line.intermediate_points:
				# 	line.intermediate_points=sorted(line.intermediate_points,key=lambda ip:line[0].point.distance(ip.point))
				
				# line.plot()
				all_line_points=point_getter(line)
				for i,line_point in enumerate(all_line_points[:-1]):
					sub_line=TheLine([line_point,all_line_points[i+1]])  #.plot(text=f'{i}')
					
					self.G.add_edge(line_point,all_line_points[i+1],
					                line=sub_line,
					                parent_line=line,
					                )
					# line_point.plot(f'LP-{i}')
					pass
		
		# plt.show()
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
			# if point.x==sorted_points[i_point-1].x: logger.warning('Points have same x vallues!!!. Wrong altitude!')
			
			if show: point.plot(f'P-{i_point}')
			# logger.debug('factor: %s',factor)
			altitude+=factor*(point.x-sorted_points[i_point-1].x)
			neighbors=adjacency_data[point]
			if all(map(lambda n:n.x>point.x,neighbors)):
				factor+=1
			elif all(map(lambda n:n.x<point.x,neighbors)):
				factor-=1
		
		if factor!=0:
			logger.warning('Final factor value is not 0, but %s',factor)
		# logger.debug('altitude: %s',altitude)
		return altitude
	
	def get_min_altitude(self,use_mp=False):
		if use_mp:
			alt_getter=lambda _exterior_line:(
				FieldPoly(shaff.rotate(self.geometry,180-_exterior_line.angle,_exterior_line[0].geometry)).get_altitude(),_exterior_line)
			altitude_data=ParallelTasker(alt_getter).set_run_params(_exterior_line=self.all_lines).run(sleep_time=1e-4)
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
		# logger.debug('exterior_line.angle: %s',exterior_line.angle)
		# plt.show()
		logger.a().debug('min_altitude: %s',min_altitude).s()
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
	# 			# logger.debug('exterior_line.angle:\n%s',exterior_line.angle)
	# 			# logger.debug('field_lines[0].angle:\n%s',field_lines[0].angle)
	# 			angle_diff=abs(exterior_line.angle-field_lines[0].angle)
	# 			# logger.debug('angle_diff: %s',angle_diff)
	# 			if angle_diff<min_dist or abs(180-angle_diff)<min_dist:
	# 				# logger.debug('angle_diff: %s',angle_diff)
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
	# 	# logger.debug('# of field lines along borders: %s',fl_counters)
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
	
	def get_field_lines_old(self,offset_distance,base_line,show=False):
		out_field_lines=list()
		shrinked_poly=self.geometry.buffer(-offset_distance/2,join_style=shg.JOIN_STYLE.mitre)
		valid_self=self.as_valid(self.geometry)
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
				offset_lines=parent_line.get_field_lines(valid_self)
				
				# offset_lines=list()
				
				offset_lines.extend(parent_line.offset_by(offset_distance,lines_count_to_cover))
				offset_lines.extend(parent_line.offset_by(-offset_distance,lines_count_to_cover))
				
				# for i in range(len(offset_lines)): offset_lines[i]=offset_lines[i].reversed()
				for offset_line in offset_lines:
					field_lines=offset_line.get_field_lines(valid_self)
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
					break
			
			if counter!=1:
				logger.warning('counter: %s',counter)
		
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
		# logger.debug('len(adjacency_data): %s',len(adjacency_data))
		
		self.add_angle_info()
		previous_subfields=list()
		for path in get_paths(self,self.adjacency_info,start_node):
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
					# logger.info('is_seen: %s',is_seen)
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
				# logger.debug('diff_result:\n%s',diff_result)
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
			# 	logger.debug('line.angle: %s',line.angle)
			# for l1,l2 in itertools.combinations(raw_poly.exterior_lines,2):
			#
			# 	result=l1.geometry.distance(l2.geometry)==0 and abs(l1.angle-l2.angle)<min_dist
			# 	if result:
			# 		logger.info('l1.angle: %s',l1.angle)
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
		# logger.warning('border_index:\n%s',border_index)
		field_lines=sorted(field_lines,key=lambda _line:_line.geometry.distance(start.geometry))
		first_field_line=TheLine(sorted(field_lines[0][:],key=lambda p:p.geometry.distance(start.geometry)))
		# logger.debug('first_field_line.angle: %s',first_field_line.angle)
		
		start_side_points=list()
		
		for i,field_line in enumerate(field_lines):
			# logger.debug('field_line.angle: %s',field_line.angle)
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
			point.plot(c='k')  # logger.warning('border_index:\n%s',border_index)
		field_lines=sorted(field_lines,key=lambda _line:_line.geometry.distance(start.geometry))
		first_field_line=TheLine(sorted(field_lines[0][:],key=lambda p:p.geometry.distance(start.geometry)))
		# logger.debug('first_field_line.angle: %s',first_field_line.angle)
		
		start_side_points=list()
		
		for i,field_line in enumerate(field_lines):
			# logger.debug('field_line.angle: %s',field_line.angle)
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
	
	def get_field_lines(self,offset_distance,show=False):
		x_min=min(self.xy[:,0])
		x_max=max(self.xy[:,0])
		bound_poly=FieldPoly(self.bounds_xy)
		the_line=bound_poly.get_exterior_lines()[-1]
		offset_lines=the_line.offset_by(offset_distance,np.ceil((x_max-x_min)/offset_distance).astype(int)-1)
		if show:
			for offset_line in offset_lines:
				offset_line.plot(c='gray')
		return offset_lines
	
	# @logger.trace()
	def get_cost(self,linewidth):
		total_cost=0
		xy=self.xy[:-1]
		for hole in self.holes:
			total_cost+=hole.get_cost(linewidth)
		sorted_inds=xy[:,0].argsort()
		sorted_xy=xy[sorted_inds]
		num_points=xy.shape[0]
		# logger.debug('num_points: %s',num_points)
		alpha=0
		counter=0
		for i in range(num_points):
			# logger.debug('alpha: %s',alpha)
			curr_x_val=sorted_xy[i,0]
			prev_x_val=sorted_xy[i-1,0]
			alpha+=counter*(curr_x_val-prev_x_val)
			vi_x=xy[sorted_inds[i]][0]
			vi_x_prev=xy[sorted_inds[i]-1][0]
			vi_x_next=xy[(sorted_inds[i]+1)%num_points][0]
			if vi_x<vi_x_prev and vi_x<vi_x_next:
				counter+=1
			# ThePoint(xy[sorted_inds[i]]).plot(lw=5,text=f'{sorted_inds[i]} +1')
			elif vi_x>vi_x_prev and vi_x>vi_x_next:
				counter-=1
		total_cost+=alpha/linewidth
		
		return total_cost
	
	def get_min_cost(self,linewidth):
		"""

		:rtype: MinCoster
		"""
		min_coster=MinCoster(np.inf,None,None)
		
		for exterior_line in self.all_lines:
			# logger.debug('exterior_line.angle: %s',exterior_line.angle)
			# self.plot()
			rotated_poly=FieldPoly(shaff.rotate(self.geometry,270-1e-6-exterior_line.angle,exterior_line[0].geometry))  #.plot('rotated_poly')
			# exterior_line.plot()
			cost=rotated_poly.get_cost(linewidth)
			step_1=rotated_poly.bounds_xy
			step_2=step_1[2]-step_1[0]
			width_to_height_ratio=step_2[0]/step_2[1]
			# logger.debug('cost: %s',cost)
			# plt.show()
			if cost<min_coster.cost:
				min_coster=MinCoster(cost,exterior_line,width_to_height_ratio)
		
		return min_coster
	
	@property
	def vis_poly(self):
		return get_vis_poly(self)
	
	@property
	def vis_env(self):
		return get_vis_env(self)
	
	@classmethod
	def from_(cls,vis_poly):
		xs=list()
		ys=list()
		for i in range(vis_poly.n()):
			xs.append(vis_poly[i].x())
			ys.append(vis_poly[i].y())
		res=np.stack([xs,ys]).T
		# logger.debug('res:\n%s',res)
		return cls(res)
	
	def get_outer_points(self,show=False):
		conv_hull=self.convex_hull
		if show: conv_hull.plot()
		outs=list()
		for point in self.points:
			if point in conv_hull:
				outs.append(point)
				if show: point.plot()
		return outs
	
	def get_visible_poly(self,observer_point,as_field=True):
		visible_poly=generate_visible_poly(observer_point,self)
		if as_field: return FieldPoly.from_(visible_poly)
		else: return visible_poly
	
	@property
	def area_cost_ratio(self):
		return self.geometry.area/self.get_min_cost(path_width).cost
	
	@logger.trace()
	def get_subparcel_centers(self,parcel_area,random_state=123,show='',grid_factor=0.1):
		n_clusters=int(self.p.area/parcel_area)
		logger.debug('n_clusters: %s',n_clusters)
		if n_clusters==0: raise NotImplementedError()
		cluster_centers=self.generate_clusters(grid_resolution=np.sqrt(parcel_area)*grid_factor,
		                                       n_clusters=n_clusters,
		                                       random_state=random_state,
		                                       show=show)
		# logger.debug('cluster_centers:\n%s',cluster_centers)
		return cluster_centers

def get_vis_poly(polygon: FieldPoly):
	return vis.Polygon([vis.Point(*xy) for xy in polygon.xy[:-1]])

def get_vis_env(polygon: FieldPoly):
	if polygon.is_clockwise: polygon=polygon.reversed
	vis_env=vis.Environment(polygon.vis_poly,*(x.vis_poly for x in polygon.holes))
	assert vis_env.is_valid(epsilon)
	return vis_env

def generate_visible_poly(observer,environment):
	if isinstance(observer,ThePoint): observer=observer.vis_observer
	if isinstance(environment,FieldPoly): environment=environment.vis_env
	
	observer.snap_to_boundary_of(environment,epsilon)
	observer.snap_to_vertices_of(environment,epsilon)
	return vis.Visibility_Polygon(observer,environment,epsilon)

def get_shortest_path_length(env,observer,end):
	return env.shortest_path(observer,end,epsilon).length()

# def get_dfs_paths(graph,_start,goal):
# 	stack=[(_start,[_start])]
# 	while_counter=itertools.count()
# 	while stack:
# 		next_while_counter=next(while_counter)
# 		# logger.debug('next_while_counter: %s',next_while_counter)
#
# 		(vertex,path)=stack.pop()
# 		for_counter=itertools.count()
# 		for next_node in graph[vertex]-set(path):
# 			next_for_counter=next(for_counter)
# 			# logger.debug('next_for_counter: %s',next_for_counter)
# 			the_path=path+[next_node]
# 			path_length=len(the_path)
# 			if path_length>2:
# 				the_poly=ThePoly(the_path)
# 				# the_poly.plot()
# 				# plt.show()
# 				# logger.debug('the_poly.geometry.area: %s',the_poly.geometry.area)
# 				if not the_poly.is_convex:
# 					continue
# 				if next_node==goal:
# 					if the_poly.geometry.area>min_dist:
# 						yield the_path
# 				else:
# 					stack.append((next_node,the_path))
# 			else:
# 				stack.append((next_node,the_path))

def dfs_paths_old(field_poly: FieldPoly,graph,start,):
	hole_points=[hole.geometry.representative_point() for hole in field_poly.holes]
	
	def get_dfs_paths(_start,goal):
		stack=[(_start,[_start])]
		while_counter=itertools.count()
		while stack:
			next_while_counter=next(while_counter)
			# logger.debug('next_while_counter: %s',next_while_counter)
			
			(vertex,path)=stack.pop()
			for_counter=itertools.count()
			for next_node in graph[vertex]-set(path):
				next_for_counter=next(for_counter)
				# logger.debug('next_for_counter: %s',next_for_counter)
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
					# logger.debug('the_poly.geometry.area: %s',the_poly.geometry.area)
					# logger.debug('contains_hole: %s',contains_hole)
					# logger.debug('the_poly.is_convex: %s',the_poly.is_convex)
					if contains_hole or (the_poly.geometry.area>min_dist and not the_poly.is_convex):
						continue
					# field_poly.plot()
					# the_poly.plot()
					# plt.show()
					# logger.debug('next_node:\n%s',next_node)
					# logger.debug('goal:\n%s',goal)
					# logger.warning('path_length: %s',path_length)
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

def dfs(graph,start,end):
	fringe=[(start,[])]
	while fringe:
		# logger.debug('fringe: %s',fringe)
		state,path=fringe.pop()
		if path and state==end:
			yield path
			continue
		
		for next_state in graph[state]:
			if next_state in path:
				continue
			fringe.append((next_state,path+[next_state]))

def bfs_paths(graph,start,goal,the_field=None):
	queue=[(start,[start])]
	show=bool(the_field)
	while queue:
		# logger.debug('len(queue): %s',len(queue))
		# logger.debug('queue: %s',queue)
		(vertex,path)=queue.pop(0)
		
		for _next in graph[vertex]-set(path):
			if _next==goal:
				if show:
					the_field.plot()
					for i,p in enumerate(path+[_next]):
						p.plot(text=str(i))
					plt.show()
				yield path+[_next]
			else:
				queue.append((_next,path+[_next]))

def get_paths(poly: FieldPoly,start,goal,loop_limit=5e6):
	graph=poly.adjacency_info
	if len(poly.G.decision_points['all'])<dfs_threshold:
		pop_index=-1
		loop_limit=5e6
	else:
		pop_index=0
	hole_points=[hole.geometry.representative_point() for hole in poly.holes]
	stack=[(start,[start])]
	seen=set()
	loop_counter=itertools.count()
	# curr_loop_count=0
	while stack:
		(vertex,path)=stack.pop(pop_index)
		curr_loop_count=next(loop_counter)
		if curr_loop_count%int(loop_limit/50)==0: logger.debug('curr_loop_count: %s',curr_loop_count)
		
		for _next in graph[vertex]-set(path):
			if _next==goal:
				some_path=path+[_next]
				if len(some_path)<3: continue
				path_hash=sum(hash(x) for x in some_path)
				if path_hash in seen: continue
				seen.add(path_hash)
				
				some_field=FieldPoly(some_path)  #.plot()
				if some_field.geometry.area<min_dist: continue
				if not some_field.geometry.is_valid: continue
				
				some_field=FieldPoly.as_valid(some_field.geometry)
				
				contains_hole=False
				for point_in_hole in hole_points:
					if point_in_hole.within(some_field.geometry):
						contains_hole=True
						break
				if contains_hole: continue
				
				pass
				yield some_field
			else:
				stack.append((_next,path+[_next]))
		
		if curr_loop_count>loop_limit:
			logger.info('Final_loop_count: %s',curr_loop_count)
			break

def get_other_polys(parent_poly,child_poly):
	try:
		diff_result=parent_poly.geometry.difference(child_poly.geometry)
	except Exception as exc:
		logger.warning(str(exc))
		return []
	if isinstance(diff_result,shg.Polygon): polygons=[diff_result]
	elif isinstance(diff_result,shg.MultiPolygon): polygons=[x for x in diff_result]
	elif isinstance(diff_result,shg.GeometryCollection):
		diff_results=diff_result
		polygons=list()
		for diff_result in diff_results:
			if isinstance(diff_result,shg.Polygon):
				polygons.append(diff_result)
			elif isinstance(diff_result,shg.MultiPolygon):
				polygons.extend([x for x in diff_result])
	else:
		logger.error('diff_result:\n%s',diff_result)
		raise NotImplementedError(str(type(diff_result)))
	
	polygons=[FieldPoly.as_valid(x) for x in polygons if x.area>min_dist]
	return polygons

Candidate=collections.namedtuple('Candidate',['acr','field'])

def decompose_from_point(field_poly,some_point,show=False):
	counter=itertools.count()
	curr_count=0
	start=some_point
	goal=field_poly.G.nodes[some_point]['lines'][0][-1]  #neighbor_point
	
	best=Candidate(field_poly.area_cost_ratio,field_poly)
	if show:
		field_poly.plot()
		logger.debug('best: %s',best)
		best.field.plot(text=f'acr = {best.acr}')
		start.plot('start')
		goal.plot('goal')
		plt.show()
	decomposed_polygons=[best.field]
	for path_field in get_paths(field_poly,start,goal,loop_limit=search_loop_limit):
		curr_count=next(counter)
		polygons=get_other_polys(field_poly,path_field)
		costs=[p.get_min_cost(path_width).cost for p in [path_field]+polygons]
		
		area_cost_ratio=field_poly.geometry.area/sum(costs)
		
		if area_cost_ratio>best.acr:
			best=Candidate(area_cost_ratio,path_field)
			logger.debug('curr_count: %s',curr_count)
			logger.debug('best: %s',best)
			decomposed_polygons=[best.field,*polygons]
		# field_poly.plot()
		# best.field.plot(text=f'acr = {best.acr}')
		# start.plot(text='Start')
		# goal.plot(text='Goal')
		# plt.show()
		
		# field_poly.plot()
		# path_field.plot(text=f'acr = {area_cost_ratio}')
		# plt.savefig(figs_folder.get_filepath('figure',ext='.png',iterated=True))
		# plt.show()
		pass
	
	logger.debug('Final_count: %s',curr_count)
	logger.info('Final best: %s',best)
	if show:
		field_poly.plot()
		best.field.plot(text=f'acr = {best.acr}')
	# plt.show()
	
	return decomposed_polygons

def decompose_from_points(field_poly: FieldPoly,points=None,use_mp=False,show=False):
	resultant_polys=[field_poly]
	max_acr=field_poly.area_cost_ratio
	logger.debug('Starting area_cost_ratio: %s',max_acr)
	points=field_poly.get_outer_points() if points is None else points
	if use_mp:
		chunks_of_polygons=ParallelTasker(decompose_from_point,field_poly,show=show)\
			.set_run_params(some_point=points)\
			.run(workers_count=workers_count)
		for res_polygons in chunks_of_polygons:
			decomp_cost=res_polygons[0].area_cost_ratio
			if decomp_cost>max_acr:
				logger.info('Better decomp acr found: %s',decomp_cost)
				max_acr=decomp_cost
				resultant_polys=res_polygons
	else:
		for outer_point in points:
			res_polygons=decompose_from_point(field_poly,outer_point,show=show)
			decomp_cost=res_polygons[0].area_cost_ratio  # we only consider the cost of the first optimum polygon.
			if decomp_cost>max_acr:
				logger.info('Better decomp acr found: %s',decomp_cost)
				max_acr=decomp_cost
				resultant_polys=res_polygons
	return resultant_polys

def perform_optimization(field_poly,use_mp=False):
	polygons=decompose_from_points(field_poly,field_poly.get_outer_points(),use_mp=False,show=False)
	optimum_polygons=[polygons[0]]
	if use_mp:
		if not len(polygons[1:])==0:
			chunks_of_optimum_sub_polygons=mi.flatten(
				ParallelTasker(perform_optimization).set_run_params(field_poly=polygons[1:]).run(len(polygons[1:])))
			optimum_polygons.extend(chunks_of_optimum_sub_polygons)
		pass
	else:
		for other_polygon in polygons[1:]:
			optimum_sub_polygons=perform_optimization(other_polygon)
			optimum_polygons.extend(optimum_sub_polygons)
	return optimum_polygons

def get_field_lines_count(field_poly: FieldPoly,show=False):
	field_poly=FieldPoly.as_valid(field_poly)
	# field_poly.plot()
	# plt.show()
	# field_lines=field_poly.get_field_lines(path_width,show=show)
	field_lines=field_poly.get_field_lines_old(path_width,field_poly.get_min_cost(path_width).base_line,show=show)
	# if show: field_poly.plot(text=f'{len(field_lines)}')
	return len(field_lines)

@logger.trace()
def main():
	# figs_folder=gsup.project_folder['figures']
	# line=shwkt.loads(r'LINESTRING (-234.4914503931494 64.05445361575346, 1555.421523437801 707.7923809846424)')
	# poly=shwkt.loads(r'POLYGON ((858.0445350926536 436.2787595905871, 874.1837236739181 456.4105901811055, 452.3998231233589 304.7170174165853, 278.4614376825613 242.1604923952041, 273.7995111651426 240.4838416542209, 105.1701033341294 179.8366777542666, 231.290717671669 252.1233683921036, 360.1479755206071 325.9785805910953, 782.4561646582736 568.0267309459517, 963.0932243188652 567.3152367102089, 858.0445350926536 436.2787595905871))')
	# FieldPoly.as_valid(poly).plot()
	# plt.show()
	# return
	
	# field_poly=FieldPoly.synthesize(cities_count=7,hole_count=0,hole_cities_count=3,seed=455960)
	# field_poly=FieldPoly.synthesize(cities_count=20,hole_count=0,hole_cities_count=3,seed=500140)
	# field_poly=FieldPoly.synthesize(cities_count=20,hole_count=0,hole_cities_count=3,seed=822514)
	# field_poly=FieldPoly.synthesize(cities_count=20,hole_count=0,hole_cities_count=5,seed=403969)
	# field_poly=FieldPoly.synthesize(cities_count=10,hole_count=0,hole_cities_count=5,seed=640166)
	# field_poly=FieldPoly.synthesize(cities_count=50,hole_count=0,hole_cities_count=5,seed=734748)
	# field_poly=FieldPoly.synthesize(cities_count=10,hole_count=0,hole_cities_count=5,seed=None)
	# field_poly=FieldPoly.synthesize(cities_count=15,hole_count=1,hole_cities_count=4,seed=753492)
	# field_poly=FieldPoly.synthesize(cities_count=10,hole_count=1,hole_cities_count=4,seed=485022)
	# field_poly=FieldPoly.synthesize(cities_count=10,hole_count=1,hole_cities_count=4,seed=None)
	# field_poly=FieldPoly.synthesize(cities_count=6,hole_count=1,hole_cities_count=5,seed=575595)
	field_poly=FieldPoly(shg.Polygon([[0,0,],
	                                  [1000,0],
	                                  [1000,1000],
	                                  [0,1000],
	                                  ])).plot()
	coordinates=field_poly.get_subparcel_centers(5*1e4,show='11')
	logger.debug('coordinates:\n%s',coordinates)
	plt.show()
	# field_poly.get_subparcel_centers()
	
	return
	for poly_extent in [8e3,15000]:
		logger.debug('poly_extent: %s',poly_extent)
		for parcel_area in [50,25,5]:
			logger.debug('parcel_area: %s',parcel_area)
			parcel_area*=1e4
			for i in range(5):
				field_poly=ParallelTasker(FieldPoly.synthesize,cities_count=50,poly_extent=poly_extent,hole_count=0)\
					.set_run_params(seed=[np.random.randint(0,1000000)]).run()[0]
				# field_poly=FieldPoly.synthesize(cities_count=10,poly_extent=poly_extent,hole_count=2)
				if field_poly.p.area/parcel_area<2: continue
				cluster_centers=field_poly.get_subparcel_centers(parcel_area,show='11',)
				plt.gca().set_aspect('equal','box')
				plt.savefig(figs_folder['cluster_centers_no_holes_2']
				            .get_filepath('clusters',
				                          poly_extent=poly_extent,
				                          parcel_area=parcel_area/1e4,
				                          ext='.png',iterated=True))
				plt.clf()
		# plt.show()
	return
	# field_poly=FieldPoly.synthesize(cities_count=20,seed=575595)
	for i in range(100):
		field_poly=ParallelTasker(FieldPoly.synthesize,cities_count=50,poly_extent=poly_extent,hole_count=0)\
			.set_run_params(seed=[np.random.randint(0,1000000)]).run()[0]
		field_poly.generate_clusters(grid_resolution=10,n_clusters=100,show='11')
		plt.show()
	return
	# logger.debug('field_poly: %s',field_poly)
	# field_poly_2=FieldPoly(field_poly.polygon.centroid)
	field_poly.plot(lw=5)
	# field_poly_2.plot()
	plt.show()
	return
	initial_lines_count=get_field_lines_count(field_poly,show=True)
	logger.info('initial_lines_count: %s',initial_lines_count)
	logger.info('Initial area_cost_ratio: %s',field_poly.area_cost_ratio)
	# field_poly.plot(f'acr: {field_poly.area_cost_ratio:.2f}')
	field_poly.plot(f'cost: {initial_lines_count}')
	
	plt.gca().set_aspect('equal','box')
	plt.savefig(figs_folder.get_filepath('optimized',ext='.png',iterated=True))
	plt.show()
	
	optimum_polygons=perform_optimization(field_poly,use_mp=False)
	
	costs=[]
	field_poly.plot()
	total_field_count=0
	for optimum_polygon in optimum_polygons:
		lines_count=get_field_lines_count(optimum_polygon,show=True)
		total_field_count+=lines_count
		# optimum_polygon.plot(f'acr: {optimum_polygon.area_cost_ratio:.2f}')
		optimum_polygon.plot(f'cost: {lines_count}')
		costs.append(optimum_polygon.get_min_cost(path_width).cost)
	final_acr=field_poly.geometry.area/sum(costs)
	logger.info('Final area_cost_ratio: %s',final_acr)
	logger.info('total_field_count: %s',total_field_count)
	plt.gca().set_aspect('equal','box')
	plt.savefig(figs_folder.get_filepath('optimized',ext='.png',iterated=True))
	plt.show()
	
	return

# vis_field=field_poly.get_visible_poly(outer_point)
# visited_points=set()
#
# for vis_vertex in vis_field.points:
# 	if not vis_vertex in field_poly: continue
# 	sub_field=vis_field.get_visible_poly(vis_vertex)
# 	vis_field=sub_field.geometry.intersection(vis_field)
#
# plt.savefig(figs_folder.get_filepath('figure',ext='.png',iterated=True))
# plt.show()
# return


if __name__=='__main__':
	main()
