import os
import numpy as np
import pandas as pd
import itertools
import collections
# import visilibity
from lgblkb_tools import geometry as gmtr
import shapely.geometry as shg
import matplotlib.pyplot as plt
def main():
	crop_field=gmtr.SpatialGeom(shg.Polygon([
		[0,10],
		[10,100],
		[60,85],
		[120,120],
		[80,130],
		[200,150],
		[150,75],
		[200,0],
		],holes=[
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
		])).plot(c='k',lw=1)
	
	for i in range(15):
		try:
			crop_field=crop_field.buffer(-2)
			if crop_field.geom_obj.is_empty:break
			crop_field.plot()
			# crop_field.get_area(3857)
			# print(crop_field.geom_obj.is_empty)
			# print(crop_field.geom_obj.is_valid)
			# if i>7:
		except AttributeError:
			print('Catched')
			break
	
	plt.show()
	pass
	
	pass

if __name__=='__main__':
	main()
