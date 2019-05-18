import os
import numpy as np
import pandas as pd
import itertools
import collections

from matplotlib import pyplot as plt
from shapely import geometry as shg

from egistic_navigation.base_geometry.geom_utils import GenericGeometry,min_dist

class ThePoint(GenericGeometry):
	
	def __init__(self,xy,**data):
		super(ThePoint,self).__init__(**data)
		if isinstance(xy,shg.Point): xy=xy.x,xy.y
		elif isinstance(xy,ThePoint): xy=xy.xy
		if len(xy)<2: raise ValueError('Invalid xy.',dict(xy=str(xy),type=str(type(xy))))
		self.xy=np.array(xy)
		self.tup_xy=tuple(xy)
		self._generate_id(self.tup_xy)
		self.__unit=None
	
	@property
	def x(self):
		return self.xy[0]
	
	@property
	def y(self):
		return self.xy[1]
	
	@property
	def point(self):
		return shg.Point(self.xy)
	
	@property
	def unit(self):
		if self.__unit is None: self.__unit=ThePoint(self.xy/np.linalg.norm(self.xy))
		return self.__unit
	
	# @property
	# def npxy(self):
	# 	return np.array(self.xy)
	
	def plot(self,text='',text_opts=None,**kwargs):
		plt.scatter(self.x,self.y,**kwargs)
		if text: plt.text(*self.xy,text,**(text_opts or {}))
		return self
	
	def __eq__(self,other):
		if isinstance(other,ThePoint):
			return self.point.distance(other.point)<min_dist
		else:
			return self.point.distance(ThePoint(other).point)<min_dist
		
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
		# simple_logger.debug('self.xy[item]: %s',self.xy[item])
		return self.xy[item]
