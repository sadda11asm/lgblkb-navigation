import matplotlib.pyplot as plt
import more_itertools as mi
import numpy as np
from geoalchemy2.shape import to_shape

import egistic_navigation.global_support as gsup
import lgblkb_tools.geometry as gmtr
from egistic_navigation.base_geometry.point_utils import ThePoint
from egistic_navigation.field_geometry.field_utils import TheField
from egistic_navigation.global_support import simple_logger
from lgblkb_tools.databases.sqla_orms import Cadastres_Info

def check_cases(seed=None,region_extent=1e6,show=False):
	for i in range(3,100):
		if show: plt.clf()
		simple_logger.info('i: %s',i)
		crop_field=TheField.from_tsp(cities_count=i,seed=seed,region_extent=region_extent)  #.plot()
		# if not crop_field.p.is_valid:
		# 	simple_logger.warning('crop_field.p.is_valid: %s',crop_field.p.is_valid)
		# 	crop_field.plot()
		# 	plt.text(*ThePoint(crop_field.p.centroid),i)
		# 	plt.show()
		crop_field.get_extension_lines(show=show)
		if show:
			crop_field.plot()
			plt.text(*ThePoint(crop_field.p.centroid),i)
			plt.show()

def choose_cadastres():
	cad_infos=gsup.mgr.session.query(Cadastres_Info).filter(Cadastres_Info.area>1e-3).limit(1000).all()
	for cad_info in cad_infos:
		simple_logger.debug('cad_info:\n%s',cad_info)
		sg=gmtr.SpatialGeom(to_shape(cad_info.geom)).convert_crs(4326,3857)
		has_interior=False
		for i in sg.geom_obj[0].interiors:
			has_interior=True
			break
		if not has_interior: continue
		sg.plot()
		# print(cad_infos)
		plt.show()

def main():
	# check_cases(seed=None,region_extent=1e2,show=False)
	
	# a=get_from_geojson(r'/home/lgblkb/kushik.geojson')[0]
	# ThePoly(a).plot()
	# plt.show()
	#
	# simple_logger.debug('a:\n%s',a)
	#
	#
	# return
	# crop_field=ThePoly.from_geojson(r'/home/lgblkb/kushik.geojson').plot()
	# for hole in crop_field.holes:
	# 	hole.plot(c='g')
	# crop_field=ThePoly.synthesize(cities_count=10,poly_extent=1000,hole_count=3,seed=None).plot()
	# crop_field=TheField.synthesize(cities_count=7,poly_extent=1000,hole_count=0,seed=4).plot()
	crop_field=TheField.synthesize(cities_count=10,poly_extent=1000,hole_count=5,seed=None).plot()
	# crop_field.get_exterior_lines(show=1)
	
	# crop_field.get_extension_lines
	
	key_points=set(mi.flatten([x[:] for x in [*crop_field.extension_lines,*crop_field.exterior_lines]]))
	simple_logger.debug('len(key_points): %s',len(key_points))
	for point in key_points:
		point.plot(c='g')
	
	# for key_point in key_points:
	
	# simple_logger.debug('key_points:\n%s',key_points)
	
	plt.show()
	
	# for i in range(100):
	# 	simple_logger.debug('i: %s',i)
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
