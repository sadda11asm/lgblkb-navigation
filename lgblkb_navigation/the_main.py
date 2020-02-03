import collections

import matplotlib.pyplot as plt
import numpy as np
from lgblkb_tools import logger

import lgblkb_navigation.global_support as gsup
from lgblkb_navigation.base_geometry.point_utils import ThePoint
from lgblkb_navigation.base_geometry.poly_utils import ThePoly
from lgblkb_navigation.field_geometry.field_utils import FieldPoly

# from lgblkb_tools.db_utils.sqla_orms import Cadastres_Info

def check_cases(seed=None,region_extent=1e6,show=False):
	for i in range(3,100):
		if show: plt.clf()
		logger.info('i: %s',i)
		crop_field=ThePoly.from_tsp(cities_count=i,seed=seed,region_extent=region_extent)  #.plot()
		# if not crop_field.p.is_valid:
		# 	logger.warning('crop_field.p.is_valid: %s',crop_field.p.is_valid)
		# 	crop_field.plot()
		# 	plt.text(*ThePoint(crop_field.p.centroid),i)
		# 	plt.show()
		crop_field.get_extension_lines(show=show)
		if show:
			crop_field.plot()
			plt.text(*ThePoint(crop_field.p.centroid),i)
			plt.show()

# def choose_cadastres():
# 	cad_infos=gsup.mgr.session.query(Cadastres_Info).filter(Cadastres_Info.area>1e-3).limit(1000).all()
# 	for cad_info in cad_infos:
# 		logger.debug('cad_info:\n%s',cad_info)
# 		sg=gmtr.SpatialGeom(to_shape(cad_info.geom)).convert_crs(4326,3857)
# 		has_interior=False
# 		for i in sg.geom_obj[0].interiors:
# 			has_interior=True
# 			break
# 		if not has_interior: continue
# 		sg.plot()
# 		# print(cad_infos)
# 		plt.show()

def save_current_figure(filepath,clear_after=True):
	plt.savefig(filepath,dpi=300,facecolor='w',edgecolor='w',
	            orientation='portrait',papertype=None,format=None,
	            transparent=False,bbox_inches=None,pad_inches=0.1,
	            frameon=None,metadata=None)
	if clear_after: plt.clf()

class SetOfFields(object):
	
	def __init__(self):
		self.solutions=collections.defaultdict(list)
	
	# @property
	# def all_fields(self):
	# 	return mi.flatten(self.solutions.values())
	
	def add(self,*input_fields,solution_cost):
		# the_field=solution_fields[0]
		# assert isinstance(the_field,ThePoly),f'{type(the_field)}'
		# logger.debug('input_fields:\n%s',input_fields)
		# for input_field in input_fields:
		# 	logger.debug('type(input_field): %s',type(input_field))
		
		are_subfields_unique=True
		for solution_fields in self.solutions[solution_cost]:
			# logger.debug('key: %s',key)
			# logger.debug('solution_fields:\n%s',solution_fields)
			# for solution_field in solution_fields:
			# 	logger.debug('type(solution_field): %s',type(solution_field))
			# 	logger.debug('solution_field:\n%s',solution_field)
			similarity_counter=0
			for i_subfield,input_field in enumerate(input_fields):
				# logger.debug('solution_fields: %s',solution_fields)
				if input_field in solution_fields:
					similarity_counter+=1
			if similarity_counter==len(solution_fields):
				are_subfields_unique=False
				return False
		
		self.solutions[solution_cost].append(input_fields)
		return True

base_folder=gsup.project_folder['results']['step_4']  #.create_iterated()
clear_figure_after_plot=True
use_mp=False
# while base_folder.children():
# 	existing_path=base_folder.path
# 	old_counter=base_folder.path.split('_')[-1]
# 	new_counter=int(old_counter)+1
# 	new_path=existing_path[:-len(old_counter)]+str(new_counter)
# 	base_folder=gsup.Folder(new_path)


def calculate_field_cost(solution_field,offset_distance,error_cost,show=False):
	try:
		# field_lines=solution_field.get_optimal_field_lines(offset_distance,with_baseline=False,show=show)
		# if field_lines is None: field_cost=error_cost
		# else: field_cost=len(field_lines)
		field_cost,_=solution_field.get_min_altitude(use_mp=use_mp)
	except Exception as exc:
		logger.warning(str(exc),exc_info=True)
		# p1.plot()
		# plt.show()
		field_cost=error_cost
	return field_cost

@logger.trace()
def get_the_best_solution(crop_field,offset_distance,save_folder,show,):
	# field_lines=crop_field.get_optimal_field_lines(offset_distance,show=show)
	# field_lines=crop_field.get_optimal_field_lines(offset_distance,show=show)
	# single_piece_cost=len(field_lines)
	single_piece_cost,_=crop_field.get_min_altitude(use_mp=use_mp)
	min_cost=single_piece_cost
	logger.info('Single_piece_cost: %s',single_piece_cost)
	# filepath=save_folder.get_filepath('initial',single_piece_cost=single_piece_cost,ext='.png')
	# save_current_figure(filepath,clear_after=clear_figure_after_plot)
	# if not clear_figure_after_plot: plt.show()
	# return
	set_of_fields=SetOfFields()
	set_of_fields.add(crop_field,solution_cost=single_piece_cost)
	
	for i_start,start_node in enumerate(crop_field.all_points):
		for solution_fields in crop_field.get_subfields(start_node,offset_distance=offset_distance,show=show):
			if len(solution_fields)==1:
				# logger.info('min_cost: %s',min_cost)
				# return min_cost,set_of_fields.solutions[min_cost][0]
				plt.clf()
				break
			cumulative_cost=0
			# cumulative_cost=sum(gsup.ParallelTasker(calculate_field_cost,offset_distance=offset_distance,error_cost=single_piece_cost*2,show=False)\
			#                     .set_run_params(solution_field=solution_fields).run(sleep_time=0.1))
			
			for solution_field in solution_fields:
				if solution_field.is_empty:
					cumulative_cost+=single_piece_cost
				else:
					cumulative_cost+=calculate_field_cost(solution_field,offset_distance,single_piece_cost*2,show=show)
					solution_field.plot()
			
			# remaining_shrinked_poly=remaining_field.geometry.buffer(-offset_distance/2,join_style=shg.JOIN_STYLE.mitre)
			# if isinstance(remaining_shrinked_poly,shg.MultiPolygon):
			# 	cumulative_cost=p1_cost
			# 	for sub_shrinked_poly in remaining_shrinked_poly:
			# 		expanded_poly=sub_shrinked_poly.buffer(offset_distance/2,cap_style=shg.CAP_STYLE.flat)
			# 		try:
			# 			sub_field_cost=len(FieldPoly(expanded_poly).get_optimal_field_lines(offset_distance,show=show))
			# 		except:
			# 			sub_field_cost=single_piece_cost*2
			# 		cumulative_cost+=sub_field_cost
			# else:
			# 	try:
			# 		remaining_cost=len(remaining_field.get_optimal_field_lines(offset_distance,show=show))
			# 	except:
			# 		remaining_cost=single_piece_cost*2
			# 	cumulative_cost=p1_cost+remaining_cost
			logger.debug('cumulative_cost: %s',cumulative_cost)
			
			filepath=save_folder.get_filepath(start_node_index=i_start,cumulative_cost=cumulative_cost,ext='.png',iterated=True)
			save_current_figure(filepath,clear_after=clear_figure_after_plot)
			if not clear_figure_after_plot: plt.show()
			
			if cumulative_cost<single_piece_cost:
				logger.info('Another solution found!')
				if cumulative_cost<min_cost: min_cost=cumulative_cost
				is_main_field_new=set_of_fields.add(*solution_fields,solution_cost=cumulative_cost)
				# logger.debug('is_main_field_new: %s',is_main_field_new)
				if is_main_field_new:
					# for sf in solution_fields: sf.plot()
					# filepath=save_folder.get_filepath(start_node_index=i_start,cumulative_cost=cumulative_cost,ext='.png')
					# save_current_figure(filepath,clear_after=clear_figure_after_plot)
					# if not clear_figure_after_plot: plt.show()
					pass
			plt.clf()
	
	# if len(save_folder.children())==1: save_folder.delete()
	
	logger.info('min_cost: %s',min_cost)
	
	return min_cost,set_of_fields.solutions[min_cost][0]

@logger.trace()
def save_solution(solution_cost,solution_fields,save_folder,offset_distance,savename='solution'):
	# plt.clf()
	for i_solution_field,solution_field in enumerate(solution_fields):
		solution_field.plot()
		solution_field.get_field_lines(offset_distance,solution_field.data.base_line,show=True)
	# field_cost=len(field_lines)
	# solution_field.plot(text=f'Field-{i_solution_field}@{field_cost}')
	
	filepath=save_folder.get_filepath(savename,solution_cost=solution_cost,ext='.png',iterated=True)
	save_current_figure(filepath,clear_after=clear_figure_after_plot)
	if not clear_figure_after_plot: plt.show()
	return filepath

@logger.trace()
def search_for_solution(crop_field,offset_distance,save_folder,show):
	solution_cost,solution_fields=get_the_best_solution(crop_field,offset_distance,save_folder,show=show)
	
	if len(solution_fields)==1:
		# solution_path=save_solution(solution_cost,solution_fields,save_folder,offset_distance,'existing_solution')
		return solution_cost,solution_fields
	else:
		first_optimal_field=solution_fields[0]
		# cumulative_cost=len(first_optimal_field.get_optimal_field_lines(offset_distance))
		cumulative_cost,_=first_optimal_field.get_min_altitude(use_mp=use_mp)
		new_fields=[first_optimal_field]
		remaining_fields=solution_fields[1:]
		# results=gsup.ParallelTasker(search_for_solution,offset_distance=offset_distance,save_folder=save_folder,show=show)\
		# 	.set_run_params(crop_field=remaining_fields).run(sleep_time=0.2)
		for remaining_field in remaining_fields:
			solution_cost,rem_sol_fields=search_for_solution(remaining_field,offset_distance,save_folder,show)
			cumulative_cost+=solution_cost
			new_fields.extend(rem_sol_fields)
		
		# for solution_cost,solution_fields,_ in results:
		# 	cumulative_cost+=solution_cost
		# 	new_fields.extend(solution_fields)
		pass
		
		# for i_remaining_field,remaining_field in enumerate(remaining_fields):
		# 	solution_cost,solution_fields,_=search_for_solution(remaining_field,offset_distance,save_folder,show)
		# 	cumulative_cost+=solution_cost
		# 	new_fields.extend(solution_fields)
		
		return cumulative_cost,new_fields

def run_single(cities_count,i_field,the_seed):
	crop_field=FieldPoly.synthesize(cities_count=cities_count,poly_extent=2000,seed=the_seed,
	                                hole_count=0,hole_cities_count=None)
	# geojson_path=r'/home/lgblkb/PycharmProjects/lgblkb_navigation/scripts/nav_test.geojson'
	# crop_field=FieldPoly.from_geojson(geojson_path)
	crop_field.plot()
	plt.show()
	crop_field.plot()
	save_folder=base_folder.create(field=i_field,cities_count=cities_count,the_seed=the_seed)
	logger.info('save_folder:\n%s',save_folder)
	
	offset_distance=24
	show=True
	
	# single_piece_cost=len(crop_field.get_optimal_field_lines(offset_distance,show=True))
	single_piece_cost,base_line=crop_field.get_min_altitude(use_mp=use_mp)
	crop_field.get_field_lines(offset_distance,base_line,show=True)
	filepath=save_folder.get_filepath('initial_solution',single_piece_cost=single_piece_cost,ext='.png')
	save_current_figure(filepath,clear_after=clear_figure_after_plot)
	if not clear_figure_after_plot: plt.show()
	
	# crop_field.get_extension_lines(show=True)
	# plt.show()
	# return
	
	solution_cost,solution_fields=search_for_solution(crop_field,offset_distance,save_folder,show,)
	solution_path=save_solution(solution_cost,solution_fields,save_folder,offset_distance,'final_solution')
	logger.info('solution_cost: %s',solution_cost)
	logger.info('Number of generated subfields: %s',len(solution_fields))
	logger.info('solution_path:\n%s',solution_path)

# solution_field,=solution_fields
# solution_field: FieldPoly=solution_field
# solution_field.plot()
# field_lines=solution_field.get_optimal_field_lines(offset_distance,show=True)
# filepath=save_folder.get_filepath('optimum',cost=len(field_lines),ext='.png')
# save_current_figure(filepath,clear_after=clear_figure_after_plot)
# if not clear_figure_after_plot: plt.show()


@logger.trace()
def main():
	# geojson_path=r'/home/lgblkb/PycharmProjects/lgblkb_navigation/scripts/nav_test.geojson'
	# crop_field=FieldPoly.from_geojson(geojson_path)
	# field_cost,base_line=crop_field.get_min_altitude(use_mp=True)
	# # crop_field.get_field_lines(12,base_line,show=True)
	# logger.debug('field_cost: %s',field_cost)
	# logger.debug('base_line: %s',base_line)
	# crop_field.plot()
	# base_line.plot(text='base_line')
	# plt.show()
	pass
	# crop_field=FieldPoly(shg.Polygon([[0,0],
	#                                   [1.1,5],
	#                                   [5,6],
	#                                   [6,2],
	#                                   [8,0],
	#                                   [7,-1],
	#                                   [4,1],
	#                                   [0,0],
	#                                   ],[[[1,1],
	#                                       [2,2],
	#                                       [2.2,1],
	#                                       [1,1],
	#                                       ],
	#                                      [[3,3],
	#                                       [5.2,4],
	#                                       [4.2,3],
	#                                       ]]))
	# crop_field.plot()
	# cost=crop_field.get_altitude()
	# logger.debug('cost: %s',cost)
	# crop_field=FieldPoly.synthesize(10,2000,0,seed=1234)
	# return
	
	
	
	# for i,sorted_point in enumerate(sorted_points):
	# 	sorted_point.plot(f'P-{i}')
	# plt.show()
	# return
	
	
	
	
	
	
	# crop_field.plot()
	# plt.show()
	pass
	# return
	run_single(10,2,np.random.randint(0,99999999))
	# run_single(15,0,93776577)
	return
	
	geojson_path=r'/home/lgblkb/PycharmProjects/lgblkb_navigation/scripts/nav_test.geojson'
	crop_field=FieldPoly.from_geojson(geojson_path)
	crop_field.plot()
	plt.show()
	show=True
	offset_distance=24
	save_folder=base_folder['nav_test']
	solution_cost,solution_fields,solution_path=search_for_solution(crop_field,offset_distance,save_folder,show,)
	logger.info('solution_cost: %s',solution_cost)
	logger.info('Number of generated subfields: %s',len(solution_fields))
	logger.info('solution_path:\n%s',solution_path)
	return
	# check_cases(seed=None,region_extent=1e2,show=False)
	
	# a=get_from_geojson(r'/home/lgblkb/kushik.geojson')[0]
	# ThePoly(a).plot()
	# plt.show()
	#
	# logger.debug('a:\n%s',a)
	#
	#
	# return
	# crop_field=ThePoly.from_geojson(r'/home/lgblkb/kushik.geojson').plot()
	# for hole in crop_field.holes:
	# 	hole.plot(c='g')
	# crop_field=ThePoly.synthesize(cities_count=10,poly_extent=1000,hole_count=3,seed=None).plot()
	# crop_field=ThePoly.synthesize(cities_count=7,poly_extent=1000,hole_count=0,seed=4).plot()
	
	# from sortedcontainers import SortedDict
	
	# d=SortedDict(c=1,b=2,a=0,d=-1)
	# print(d)
	
	# return
	field_xy=np.array([[1,1],
	                   [2,2],
	                   [4,2],
	                   # [0,5],
	                   # [5,0],
	                   [1,1],
	                   ])
	field_xy=np.array([[0,0],
	                   [0,5],
	                   [5,5],
	                   [5,10],
	                   [7,8],
	                   [7,0],
	                   [5,0],
	                   [5,1],
	                   [0,0],
	                   ])
	
	# crop_field=FieldPoly(field_xy).plot(alpha=0.2)
	# for point in crop_field.points:
	# 	point.plot()
	# 	ThePoly(point.geometry.buffer(1).envelope).plot()
	
	# crop_field.buffer(1,join_style=shg.JOIN_STYLE.mitre).plot()
	# crop_field.buffer(-0.5,join_style=shg.JOIN_STYLE.mitre).plot()
	# plt.show()
	# return
	
	# crop_field=FieldPoly.synthesize(cities_count=10,poly_extent=1000,hole_count=0,seed=8).plot()
	# crop_field=FieldPoly.synthesize(cities_count=10,poly_extent=1000,hole_count=1,seed=8).plot()
	# crop_field=FieldPoly.synthesize(cities_count=6,poly_extent=1000,hole_count=0,seed=123858).plot()
	# crop_field=FieldPoly.synthesize(cities_count=4,poly_extent=1000,hole_count=2,seed=1).plot()
	# crop_field=FieldPoly.synthesize(cities_count=4,poly_extent=1000,hole_count=2,seed=3).plot()
	# crop_field=FieldPoly.synthesize(cities_count=50,poly_extent=1000,hole_count=10,seed=1234).plot()
	# crop_field=FieldPoly.synthesize(cities_count=50,poly_extent=1000,hole_count=5,seed=2).plot()
	# crop_field=FieldPoly.synthesize(cities_count=50,poly_extent=1000,hole_count=10).plot()
	pass
	# seeds=[338799,290719,699934]
	# while True:
	# crop_field=FieldPoly.synthesize(cities_count=20,poly_extent=1000,hole_count=0,seed=338799).plot()
	# crop_field.get_extension_lines(show=True)
	#
	# decision_points=crop_field.decision_points
	# crop_field.generate_decision_points(show=True)
	# crop_field.generate_adjacency_info()
	# for l1,l2 in itertools.combinations(crop_field.exterior_lines,2):
	# 	distance=l1.geometry.distance(l2.geometry)
	# 	logger.debug('distance: %s',distance)
	# return
	# plt.show()
	# return
	# for i,point in enumerate(crop_field.all_points):
	# 	point.plot(i)
	
	# for node,neighbors_data in crop_field.G.adjacency():
	# 	logger.info('node: %s',node)
	# 	for neighbor_node in neighbors_data:
	# 		logger.debug('neighbor_node.xy: %s',neighbor_node.xy)
	# 		pass
	# return
	# plt.show()
	# return
	pass
	# for i_main,(node,node_data) in enumerate(crop_field.adjacency_info.items()):
	# 	crop_field.plot()
	# 	for extension_line in crop_field.extension_lines:extension_line.plot(alpha=0.3)
	# 	for point in crop_field.decision_points['all']: point.plot()
	#
	# 	for i_neighbor,neighbor_node in enumerate(node_data):
	# 		neighbor_node: ThePoint=neighbor_node
	# 		node.plot(f'{i_main}')
	# 		neighbor_node.plot(f'{i_main}-{i_neighbor}')
	# 	plt.show()
	# 	# logger.debug('neighbor_node.id: %s',neighbor_node.id)
	# 	# return
	# for cities_count in range(10,51,5):
	pass
	run_single(50,0,np.random.randint(0,99999999))
	# run_single(4,0,79838627)
	# run_single(4,0,51194152)
	
	return
	task_count=10
	gsup.ParallelTasker(run_single,cities_count=20)\
		.set_run_params(i_field=range(task_count),
	                    the_seed=np.random.randint(0,9999999,task_count))\
		.run()
	
	# for folderpath in base_folder.children():
	# 	sub_folder=gsup.Folder(folderpath)
	# 	if len(sub_folder.children())<2: sub_folder.delete()
	
	# base_line.plot(text='BaseLine')
	# base_line[0].plot('S')
	# base_line[1].plot('F')
	
	# points_data=collections.OrderedDict()
	# for field_line in field_lines:
	# 	sorted_points=sorted(field_line[:],key=lambda p:p.geometry.distance(start_node.geometry))
	# 	points_data[field_line]=sorted_points
	#
	# start_lines=set()
	# finish_lines=set()
	#
	# for (start_point,finish_point) in points_data.values():
	# 	start_finish_done=[False,False]
	# 	for line in p1.exterior_lines:
	# 		if line.almost_touches(start_point):
	# 			start_lines.add(line)
	# 			start_finish_done[0]=True
	# 		elif line.almost_touches(finish_point):
	# 			finish_lines.add(line)
	# 			start_finish_done[1]=True
	# 		if all(start_finish_done): break
	# for i, start_line in enumerate(start_lines):start_line.plot(text=f'S{i}')
	# for i, finish_line in enumerate(finish_lines):finish_line.plot(text=f'F{i}')
	#
	
	# dist_sorted_lines=sorted(field_lines,lambda line:line.geometry.)
	
	# logger.debug('node.xy: %s',node.xy)
	
	# logger.debug('path:\n%s',path)
	
	# plt.show()
	
	# for node,data in crop_field.G.nodes.data():
	# logger.debug('node: %s',node)
	# logger.debug('data: %s',data)
	# crop_field.get_exterior_lines(show=1)
	
	# for point in crop_field.G.nodes[decision_points['inner'][0]]['lines'][1].get_all_points:
	# for point in crop_field.G.inner_nodes[0]['lines'][1].get_all_points:
	# for border_line in crop_field.G.border_nodes[0,'lines']:
	# 	border_line.plot()
	# pass
	# for line in crop_field.G.inner_nodes[1,'lines']:
	# 	# logger.debug('type(line): %s',type(line))
	# 	line.plot()
	# logger.debug('line:\n%s',line)
	
	pass
	# logger.debug('target_line:\n%s',target_line)
	# logger.debug('lines:\n%s',lines)
	# for i,line in enumerate(lines): line.plot(text=i+1)
	# node_data=crop_field.G.nodes[crop_field[0][0]]
	# logger.debug('node_data: %s',node_data)
	# edge_data=crop_field.G.edges[crop_field[0][0],crop_field[-1][0]]
	# logger.debug('edge_data: %s',edge_data)
	# edge_data=crop_field.G.edges[crop_field[-1]]
	# logger.debug('edge_data: %s',edge_data)
	
	# for key_point in main_decision_points:
	
	# logger.debug('main_decision_points:\n%s',main_decision_points)
	pass
	# plt.show()
	pass
	# for i in range(100):
	# 	logger.debug('i: %s',i)
	# 	crop_field=ThePoly.synthesize(cities_count=6,poly_extent=1000,hole_count=0,seed=i).plot()
	# 	crop_field.get_extension_lines(show=True)
	#
	# 	plt.show()
	
	# crop_field=ThePoly.from_tsp(cities_count=27,seed=2,region_extent=1000,id='crop_field').plot()
	# crop_field.get_extension_lines(show=True)
	
	return
	
	points=np.array([[0,0],
	                 [0,5],
	                 [2,5],
	                 [2,2],
	                 [4,2],
	                 [4,0],
	                 [0,0],
	                 ])
	points=np.array([[0,0],
	                 [0,5],
	                 [2,5],
	                 [2.5,2.5],
	                 [4,2],
	                 [4,0],
	                 [0,0],
	                 ])
	points=np.array([[0,0],
	                 [-1,0],
	                 [0.5,1],
	                 [1,2],
	                 [1.5,1.5],
	                 [1.5,1],
	                 [2,0.75],
	                 [3,1],
	                 [2,-0.5],
	                 [0,-1],
	                 [0,0],
	                 ])
	unholy_field=ThePoly(points).plot()
	unholy_field.get_extension_lines(show=True)
	unholy_field
	plt.show()
	
	pass

if __name__=='__main__':
	main()
