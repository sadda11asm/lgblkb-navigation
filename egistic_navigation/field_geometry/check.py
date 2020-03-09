from egistic_navigation.field_geometry.field_utils import FieldPoly
import shapely.geometry as shg
from egistic_navigation.global_support import simple_logger
import matplotlib.pyplot as plt


logger = simple_logger

field_poly=FieldPoly(shg.Polygon([[0,0,],
	                                  [1000,0],
	                                  [1000,1000],
	                                  [0,1000],
	                                  ])).plot()

coordinates=field_poly.get_subparcel_centers(5*1e4,show='11')
logger.debug('coordinates:\n%s',coordinates)
plt.show()