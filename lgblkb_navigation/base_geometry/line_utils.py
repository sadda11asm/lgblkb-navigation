import functools

import numpy as np
from lgblkb_tools import geometry as gmtr
from lgblkb_tools import logger
from matplotlib import pyplot as plt
from shapely import geometry as shg
from sortedcontainers import SortedDict

from lgblkb_navigation.base_geometry.geom_utils import GenericGeometry,line_xy,perpendicular,normalize,min_dist
from lgblkb_navigation.base_geometry.point_utils import ThePoint

class TheLine(GenericGeometry):

	def __init__(self,coordinates=None,width=2.0,show='',line=None,text=None,**data):
		super(TheLine,self).__init__(**data)
		if isinstance(coordinates,shg.LineString): line=coordinates
		elif isinstance(coordinates,TheLine): line=coordinates.line
		elif isinstance(coordinates,np.ndarray):
			assert coordinates.shape[0]>1
		elif isinstance(coordinates,list):
			assert len(coordinates)>1
		else:
			raise NotImplementedError(f'type = {type(coordinates)}, value={coordinates}')
		try:
			self.line=line or shg.LineString(coordinates=coordinates)
		except AssertionError:
			logger.debug('coordinates:\n%s',coordinates)
			raise
		# self.line=get_rounded(self.line,decimals=round_decimals)
		# self._generate_id()
		self.width=width
		self._coverage=None
		self._xy=None
		self._vector=None
		self._unit_vector=None
		self._slope=None
		self._midpoint=None
		self._perp_unit_vector=None
		self._cover_line=None
		self._show=show
		self.text=text
		self.intermediate_points=SortedDict()
		# if show: self.plot(show,**plot_kwargs)
		pass

	@property
	def geometry(self):
		return self.line

	def get_all_points(self):
		# return [self[0],*self.intermediate_points,self[1]]
		return [self[0],*self.intermediate_points,self[1]]
		pass

	@property
	def coverage(self):
		if self._coverage is None:
			self._coverage=self.line.buffer(self.width,cap_style=shg.CAP_STYLE.round)
		return self._coverage

	def plot(self,showcode='',show_self=1,show_normal=0,show_coverage=0,show_text=None,text='',**kwargs):
		get_showcode=lambda:[int(x) if x is not None else None for x in showcode]
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
		if (text or self.text) and show_text is None:
			# p1=plt.gca().transData.transform_point(self.xy[0])
			# p2=plt.gca().transData.transform_point(self.xy[1])
			# dy=abs(p2[1]-p1[1])
			# dx=abs(p2[0]-p1[0])
			# rotn=np.degrees(np.arctan2(dy,dx))
			# trans_angle=plt.gca().transData.transform_angles(
			# 	np.array((np.arctan(self.slope),)),self.midpoint.reshape((1,2)),radians=True)[0]
			# a=np.array(self.line.interpolate(0.5,normalized=True).xy)
			# a=a+(self.perp*3).reshape(a.shape)
			# print(a)
			# print((np.abs(self.perp)*5).reshape(a.shape))
			# plt.annotate(text or self.text,a,rotation=trans_angle*180/np.pi,rotation_mode='anchor')
			plt.text(*ThePoint(self.line.centroid),text or self.text)
		return self

	def get_normal_line(self,as_the_line=True,width=None):
		normal_vector=self.perp*(width or self.width)
		normal_line=np.array([self.midpoint-normal_vector,self.midpoint+normal_vector])
		return TheLine(normal_line,width=self.width) if as_the_line else normal_line

	def get_cover_line(self,cover_line_length):
		return self.get_normal_line().get_normal_line(width=cover_line_length)

	def get_field_lines(self,field_polygon,as_the_line=True):
		field_polygon=field_polygon.as_valid(field_polygon.geometry)
		some_line=self.get_cover_line(field_polygon.cover_line_length).line
		try:
			# field_polygon.plot()
			# plt.show()

			lines=field_polygon.polygon.intersection(some_line)
		except Exception as exc:
			logger.exception(str(exc))
			logger.debug('some_line:\n%s',some_line)
			logger.debug('field_polygon.polygon:\n%s',field_polygon.polygon)

			field_polygon.plot(lw=5,c='r')
			self.get_cover_line(field_polygon.cover_line_length).plot(lw=5,c='k')
			plt.show()
			# from lgblkb_navigation.base_geometry.poly_utils import ThePoly
			# field_polygon:ThePoly=field_polygon
			# lines=field_polygon.as_valid(field_polygon).polygon.buffer(1).buffer(-1).intersection(self.get_cover_line(field_polygon.cover_line_length).line)
			raise exc

		if as_the_line:
			liner=functools.partial(TheLine,width=self.width)
		else:
			liner=lambda x:x

		if lines.is_empty: return []
		elif isinstance(lines,shg.MultiLineString):
			return [liner(x) for x in lines]
		elif isinstance(lines,shg.LineString):
			try:
				return [liner(lines)]
			except:
				logger.debug('lines: %s',lines)
				raise
		else:
			raise NotImplementedError(f"type(lines) = {type(lines)}, lines={lines}")

	def reversed(self):
		return TheLine([self[1],self[0]])

	@property
	def vector(self):
		if self._vector is None: self._vector=self[1]-self[0]
		return self._vector

	@property
	def unit_vector(self):
		if self._unit_vector is None: self._unit_vector=self.vector.xy/np.linalg.norm(self.xy)
		return self._unit_vector

	@property
	def xy(self):
		if self._xy is None:
			self._xy=line_xy(self.line)
		return self._xy

	@property
	def slope(self):
		if self._slope is None:
			self._slope=1/np.divide(*(self.xy[1,:]-self.xy[0,:]))
		return self._slope

	@property
	def perp(self):
		if self._perp_unit_vector is None:
			self._perp_unit_vector=perpendicular(normalize(self.xy[1,:]-self.xy[0,:]))
		return self._perp_unit_vector

	@property
	def midpoint(self):
		if self._midpoint is None:
			self._midpoint=np.sum(self.xy,0)/2
		return self._midpoint

	@property
	def line_points(self):
		return [*self[:],*self.intermediate_points]

	def get_along_normal(self,distance,point_on_line=None):
		point_on_line=self.midpoint if point_on_line is None else point_on_line
		return point_on_line+self.perp*distance

	def offset_by(self,distance,count=1):
		offset_lines=list()
		for i in range(count):
			points=self.get_along_normal(distance=distance*(1+i),point_on_line=self.xy)
			line=TheLine(coordinates=points,width=self.width)
			offset_lines.append(line)
		return offset_lines

	def __eq__(self,other):
		if np.linalg.norm(self.xy-other.xy)<min_dist or np.linalg.norm(self.xy-other.xy[::-1])<min_dist: return True
		else: return False

	def touches(self,xy):
		return self.line.touches(shg.Point(xy))

	def __len__(self):
		return len(self.xy)

	def __getitem__(self,item):
		if isinstance(item,slice):
			return [self[ii] for ii in range(*item.indices(len(self)))]
		# logger.debug('self.xy: %s',self.xy)
		# logger.debug('item: %s',item)
		# logger.debug('self.xy[item]: %s',self.xy[item])
		return ThePoint(self.xy[item])

	def __iter__(self):
		return iter([self[i] for i in range(2)])

	def extend_from(self,xy,length,show=False,**plot_kwargs):
		# ThePoint([0,0]).plot('Origin')
		# target_point=ThePoint(crop_field.xy[0]).plot()
		# target_line=TheLine([[0,50],[100,50]]).plot(text='target_line')
		# target_line=TheLine([[0,50],[-100,50]]).plot(text='target_line')
		# target_line=TheLine([[0,50],[0,100]]).plot(text='target_line')
		# target_line=TheLine([[0,50],[0,-100]]).plot(text='target_line')
		# target_point=ThePoint(target_line[0]).plot('Target')
		# target_point=ThePoint(self[0])  #.plot('Target')
		xy_point=ThePoint(xy)
		# if xy_point.point.distance(self[0].point)<min_dist:
		if xy_point.almost_touches(self[0]):  #.point.distance(self[0].point)<min_dist:
			target_point=self[0]
			line_vector=self[1]-self[0]
			_power=0
		# elif xy_point.point.distance(self[1].point)<min_dist:
		elif xy_point.almost_touches(self[1]):  #point.distance(self[1].point)<min_dist:
			target_point=self[1]
			line_vector=self[0]-self[1]
			_power=1
		else:
			message='xy_point does not correspond to any of the line vertices.'
			logger.error(message)
			# xy_point.plot('xy_point')
			# self.plot(text='The line')
			# plt.show()
			raise ValueError(message,dict(xy_point=str(xy_point),the_line=str(self)))

		# logger.debug('target_line.slope: %s',target_line.slope)
		# logger.debug('self.slope: %s',self.slope)
		# line_vector.plot('line_vector')
		# target_point.plot('Target')
		# self[0].plot('Start')
		# self[1].plot('End')
		# plt.show()
		if np.isposinf(self.slope) or np.isneginf(self.slope):
			# vector_point=ThePoint([0,(-1)**(np.isneginf(self.slope))*length])*(-1)**(self.vector.y>0)
			vector_point=ThePoint([0,(-1)**_power*length])  #*(-1)**(self.vector.y>0)
		# vector_point.plot('Vector point')
		else:
			vector_point=ThePoint([length,self.slope*length]).unit*length  #.plot('Vector point')
		# line_vector.plot(text='line_vector')
		new_point=(target_point+((-1)**(line_vector.x>0))*vector_point)  #.plot('New point')
		extension_line=TheLine([target_point.xy,new_point.xy])  #.plot(text='extension_line')
		if show: extension_line.plot(**plot_kwargs)
		# plt.show()
		return extension_line

	def __contains__(self,item):
		# logger.debug('item: %s',item)
		if not isinstance(item,ThePoint): item=ThePoint(item)
		for point in self[:]:
			# logger.debug('type(point): %s',type(point))
			# logger.debug('point: %s',point)
			if point==item: return True
		return False

	def __hash__(self):
		return hash(self.geometry)

	@property
	def angle(self):
		return np.arctan2(self.vector.y,self.vector.x)*180/np.pi

# def __iter__(self):
# 	return iter(self.xy)
