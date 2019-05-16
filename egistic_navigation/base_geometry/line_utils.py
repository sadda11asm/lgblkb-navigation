import numpy as np
from matplotlib import pyplot as plt
from shapely import geometry as shg

from egistic_navigation.base_geometry.geom_utils import GenericGeometry,line_xy,perpendicular,normalize,min_dist
from egistic_navigation.base_geometry.point_utils import ThePoint
from egistic_navigation.global_support import simple_logger
from lgblkb_tools import geometry as gmtr

class TheLine(GenericGeometry):
	
	def __init__(self,coordinates=None,width=2.0,show='',line=None,text=None,**box_kwargs):
		super(TheLine,self).__init__(**box_kwargs)
		self.line=line or shg.LineString(coordinates=coordinates)
		# self.line=get_rounded(self.line,decimals=round_decimals)
		self._generate_id(self.line)
		self.width=width
		self.__coverage=None
		self.__xy=None
		self.__vector=None
		self.__unit_vector=None
		self.__slope=None
		self.__midpoint=None
		self.__perp_unit_vector=None
		self.__cover_line=None
		self.__show=show
		self.text=text
		# if show: self.plot(show,**plot_kwargs)
		pass
	
	@property
	def coverage(self):
		if self.__coverage is None:
			self.__coverage=self.line.buffer(self.width,cap_style=shg.CAP_STYLE.round)
		return self.__coverage
	
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
		return TheLine(normal_line,width=self.width,show=self.__show) if as_the_line else normal_line
	
	def get_cover_line(self,cover_line_length):
		return self.get_normal_line().get_normal_line(width=cover_line_length)
	
	def get_field_lines(self,field_polygon,as_the_line=True):
		lines=field_polygon.polygon.intersection(self.get_cover_line(field_polygon.cover_line_length).line)
		if as_the_line:
			liner=lambda x:TheLine(line=x,width=self.width,show=self.__show)
		else:
			liner=lambda x:x
		
		if lines.is_empty: return None
		elif isinstance(lines,shg.MultiLineString):
			return [liner(x) for x in lines]
		else:
			return [liner(lines)]
	
	@property
	def vector(self):
		if self.__vector is None: self.__vector=self.xy[1]-self.xy[0]
		return self.__vector
	
	@property
	def unit_vector(self):
		if self.__unit_vector is None: self.__unit_vector=self.vector/np.linalg.norm(self.xy)
		return self.__unit_vector
	
	@property
	def xy(self):
		if self.__xy is None:
			self.__xy=line_xy(self.line)
		return self.__xy
	
	@property
	def slope(self):
		if self.__slope is None:
			self.__slope=1/np.divide(*(self.xy[1,:]-self.xy[0,:]))
		return self.__slope
	
	@property
	def perp(self):
		if self.__perp_unit_vector is None:
			self.__perp_unit_vector=perpendicular(normalize(self.xy[1,:]-self.xy[0,:]))
		return self.__perp_unit_vector
	
	@property
	def midpoint(self):
		if self.__midpoint is None:
			self.__midpoint=np.sum(self.xy,0)/2
		return self.__midpoint
	
	def get_along_normal(self,distance,point_on_line=None):
		point_on_line=self.midpoint if point_on_line is None else point_on_line
		return point_on_line+self.perp*distance
	
	def offset_by(self,distance,count=1):
		offset_lines=list()
		for i in range(count):
			points=self.get_along_normal(distance=distance*(1+i),point_on_line=self.xy)
			line=TheLine(coordinates=points,width=self.width,show=self.__show)
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
		# simple_logger.debug('self.xy: %s',self.xy)
		# simple_logger.debug('item: %s',item)
		# simple_logger.debug('self.xy[item]: %s',self.xy[item])
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
		if xy_point.g.distance(shg.Point(self[0]))<min_dist:
			target_point=self[0]
			line_vector=self[1]-self[0]
		elif xy_point.g.distance(shg.Point(self[1]))<min_dist:
			target_point=self[1]
			line_vector=self[0]-self[1]
		else:
			message='xy_point does not correspond to any of the line vertices.'
			simple_logger.error(message)
			xy_point.plot('xy_point')
			self.plot(text='The line')
			plt.show()
			raise ValueError(message,dict(xy_point=str(xy_point),the_line=str(self)))
		
		# simple_logger.debug('target_line.slope: %s',target_line.slope)
		if np.isposinf(self.slope) or np.isneginf(self.slope):
			vector_point=ThePoint([0,length])*(-1)**(self.vector[1]>0)
		else:
			vector_point=ThePoint([length,self.slope*length]).unit*length  #.plot('Vector point')
		# line_vector.plot(text='line_vector')
		new_point=(target_point+((-1)**(line_vector.x>0))*vector_point)  #.plot('New point')
		extension_line=TheLine([target_point.xy,new_point.xy])  #.plot(text='extension_line')
		if show:
			extension_line.plot(**plot_kwargs)
		return extension_line
	
	def __contains__(self,item):
		# simple_logger.debug('item: %s',item)
		if not isinstance(item,ThePoint): item=ThePoint(item)
		for point in self[:]:
			# simple_logger.debug('type(point): %s',type(point))
			# simple_logger.debug('point: %s',point)
			if point==item: return True
		return False
	
	# def __iter__(self):
	# 	return iter(self.xy)
	pass
