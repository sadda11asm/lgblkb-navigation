import numpy as np
import requests
import shapely.geometry as shg
from box import Box
from lgblkb_tools import logger
from lgblkb_navigation.base_geometry.line_utils import TheLine
from lgblkb_navigation.base_geometry.poly_utils import ThePoly
import matplotlib.pyplot as plt
from lgblkb_tools.geometry import SpatialGeom,convert_crs

def main():
	with open("/home/lgblkb/PycharmProjects/Egistic/Scripts/polygon.xml") as fd:
		xml=fd.read()
	headers={'content-type':'application/xml'}
	json_data=Box(requests.post(data=xml,url="https://geo.egistic.kz/geoserver/wps",headers=headers).json())
	# logger.debug('json_data.keys():\n%s',json_data.keys())
	lines_count=i_feature=0
	for i_feature,feature in enumerate(json_data.features):
		# logger.debug('feature: %s',feature)
		# logger.debug('feature.geometry: %s',feature.geometry)
		linestring=shg.shape(feature.geometry)
		line=TheLine(linestring)  #.plot()
		if len(line)<4: continue
		elif line[0]!=line[-1]:
			lines_count+=1
			# line.plot()
			continue
		# print('Aha')
		# logger.debug('len(line): %s',len(line))
		ThePoly(line.xy).plot(lw=0.7)
	logger.info('Total number of features: %s',i_feature)
	logger.debug('lines_count: %s',lines_count)
	conversion_percentage=(i_feature-lines_count)/i_feature*100
	logger.debug('conversion_percentage: %.3f%%',conversion_percentage)
	
	plt.show()
	
	
	
	pass

if __name__=='__main__':
	main()
