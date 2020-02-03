import numpy as np
import visilibity as vis
from matplotlib import pyplot as plt
from shapely import geometry as shg

from lgblkb_navigation.base_geometry.geom_utils import GenericGeometry

class ThePoint(GenericGeometry):
	
	@property
	def geometry(self):
		return self.point
	
	def __init__(self,xy,**data):
		super(ThePoint,self).__init__(**data)
		if isinstance(xy,shg.Point): xy=xy.x,xy.y
		elif isinstance(xy,ThePoint): xy=xy.xy
		# self.tup_xy=tuple(xy)
		if len(xy)<2: raise ValueError('Invalid xy.',dict(xy=str(xy),type=str(type(xy))))
		# self.xy=np.array(xy)
		self.__xy=xy
		
		# self._generate_id(self.tup_xy)
		self.__unit=None
		self.associated_lines=list()
	
	def associate_with_lines(self,*lines):
		self.associated_lines.extend(lines)
		return self
	
	@property
	def xy(self):
		return self.__xy
	
	@property
	def tup_xy(self):
		return tuple(self.xy)
	
	@property
	def x(self):
		return self.xy[0]
	
	@property
	def y(self):
		return self.xy[1]
	
	@property
	def point(self):
		# logger.debug('self.xy: %s',self.xy)
		return shg.Point(self.xy)
	
	@property
	def unit(self):
		if self.__unit is None: self.__unit=ThePoint(self.xy/np.linalg.norm(self.xy))
		return self.__unit
	
	# @property
	# def npxy(self):
	# 	return np.array(self.xy)
	
	@property
	def vis_observer(self):
		return vis.Point(*self.xy)
	
	def plot(self,text='',text_opts=None,**kwargs):
		plt.scatter(self.x,self.y,**kwargs)
		if text: plt.text(*self.xy,text,**(text_opts or {}))
		return self
	
	def __eq__(self,other):
		if isinstance(other,ThePoint):
			# return self.point.distance(other.point)<min_dist
			return self.almost_touches(other)
		else:
			return self.almost_touches(ThePoint(other))  #.point.distance(ThePoint(other).point)<min_dist
		
		# elif isinstance(other,ThePoint):
		# 	return self.tup_xy==other.tup_xy
		# else:
		# 	return self.tup_xy==tuple(other)
		pass
	
	def __mul__(self,other):
		return ThePoint(other*self.xy)
	
	def __rmul__(self,other):
		return ThePoint(other*self.xy)
	
	def __hash__(self):
		return hash(self.tup_xy)
	
	# def __repr__(self):
	# 	return f"ThePoint {self.x,self.y}"
	
	def __add__(self,other):
		if isinstance(other,ThePoint):
			return ThePoint(self.xy+other.xy)
		else:
			return ThePoint(self.xy+other)
	
	def __sub__(self,other):
		return ThePoint(self.xy-other.xy)
	
	def __iter__(self):
		return iter(self.xy)
	
	def __len__(self):
		return len(self.tup_xy)
	
	def __getitem__(self,item):
		if isinstance(item,slice):
			return [self[ii] for ii in range(*item.indices(len(self.xy)))]
		# logger.debug('self.xy[item]: %s',self.xy[item])
		return self.xy[item]
	
	def __gt__(self,other):
		other_associated_lines=other.associated_lines
		for line in self.associated_lines:
			if line in other_associated_lines:
				return line[0].point.distance(self.point)>line[0].point.distance(other.point)
