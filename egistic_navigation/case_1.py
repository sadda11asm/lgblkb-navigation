import time
import shapely.affinity as shaff
import math
from typing import List
import shapely.ops as shops
import scipy.spatial
import matplotlib.pyplot as plt
import os
import numpy as np
import pandas as pd
import itertools
import collections
import cma
import shapely.geometry as shg
import lgblkb_tools.geometry as gmtr
from egistic_navigation.global_support import simple_logger,with_logging
import dubins
from egistic_navigation.katana_case import katana

def perpendicular(a):
	b=np.empty_like(a)
	b[0]=-a[1]
	b[1]=a[0]
	return b

def normalize(a):
	a=np.array(a)
	return a/np.linalg.norm(a)

def plot_line(line):
	return plt.plot(*line.xy,lw=5)

class TheLine(object):
	
	def __init__(self,coordinates=None,width=2.0,show='',line=None):
		self.line=line or shg.LineString(coordinates=coordinates)
		self.width=width
		self.coverage=self.line.buffer(width,cap_style=shg.CAP_STYLE.round)
		self.__xy=None
		self.__slope=None
		self.__midpoint=None
		self.__perp_unit_vector=None
		self.__cover_line=None
		self.__show=show
		if show: self.plot(show)
	
	def plot(self,showcode='',show_self=1,show_normal=0,show_coverage=0,**kwargs):
		if showcode:
			if len(showcode)==1:
				show_self,show_normal,show_coverage=int(showcode),0,0
			elif len(showcode)==2:
				show_self,show_normal,show_coverage=int(showcode[0]),int(showcode[1]),0
			else:
				show_self,show_normal,show_coverage=[int(x) for x in showcode]
		else:
			show_self,show_normal,show_coverage=show_self,show_normal,show_coverage
		if show_self: plt.plot(*self.line.xy,**dict(dict(c='green'),**kwargs))
		if show_normal:
			norm_line=self.get_normal_line(as_the_line=False)
			plt.plot(norm_line[:,0],norm_line[:,1],c='y')
		if show_coverage:
			gmtr.plot_patches([self.coverage],c='green',alpha=0.1)
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
	def xy(self):
		if self.__xy is None:
			self.__xy=np.array(self.line.xy).T
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
	
	def __repr__(self):
		return "TheLine: "+str(self.line)

def get_lines(*polygons):
	lines=list()
	for pol in [polygons]:
		boundary=pol.boundary
		if boundary.type=='MultiLineString':
			for line in boundary:
				lines.append(line)
		else:
			lines.append(boundary)
	return lines

class ThePoly(object):
	
	def __init__(self,polygon):
		self.polygon: shg.Polygon=polygon
		self.__xy=None
		self.__cover_line_length=None
	
	def plot(self):
		gmtr.plot_polygon(self.polygon,c='gray')
		return self
	
	def buffer(self,distance,resolution=16,quadsegs=None,cap_style=shg.CAP_STYLE.round,
	           join_style=shg.JOIN_STYLE.round,mitre_limit=5.0):
		return ThePoly(self.polygon.buffer(distance,resolution=resolution,quadsegs=quadsegs,
		                                   cap_style=cap_style,join_style=join_style,mitre_limit=mitre_limit))
	
	@property
	def xy(self):
		if self.__xy is None:
			self.__xy=np.array(self.polygon.boundary.xy).T
		return self.__xy
	
	def get_lines(self,show=0):
		lines=list()
		current_chunk=self.polygon.boundary
		for xy in self.xy[1:-1]:
			line,current_chunk=shops.split(current_chunk,shg.Point(xy))
			lines.append(line)
		lines.append(current_chunk)
		if show:
			for line in lines:
				plt.plot(*line.xy,lw=5)
		# plt.plot(*line.parallel_offset(5).xy)
		return lines
	
	@property
	def cover_line_length(self):
		if self.__cover_line_length is None:
			xy=ThePoly(self.polygon.envelope).xy
			delta=xy[2]-xy[0]
			self.__cover_line_length=np.linalg.norm(delta)
		return self.__cover_line_length

def main():
	crop_field=ThePoly(shg.Polygon([[0,0],[10,100],[90,110],[80,130],[120,120],[150,-10]]))
	
	# plt.axis('equal')
	# plt.show()
	# gmtr.plot_polygon(crop_field.polygon.envelope)
	
	# return
	# line=TheLine([[10,5],[55,20]],show='111')
	borderlines=[TheLine(line=x,show='',width=4.5) for x in crop_field.get_lines(show=0)]
	
	data=collections.defaultdict(list)
	offset_distance=9
	for i,borderline in enumerate(borderlines):
		counter=0
		offset_lines=borderline.get_field_lines(crop_field.buffer(1e-9,cap_style=shg.CAP_STYLE.square))
		offset_lines.extend(borderline.offset_by(offset_distance,int(np.ceil(crop_field.cover_line_length/offset_distance))))
		offset_lines.extend(borderline.offset_by(-offset_distance,int(np.ceil(crop_field.cover_line_length/offset_distance))))
		# for line in offset_lines:
		# 	line.plot('100',c='magenta',alpha=0.5)
		for offset_line in offset_lines:
			field_lines=offset_line.get_field_lines(crop_field)
			if not field_lines:
				continue
			for field_line in field_lines:
				if not field_line: continue
				counter+=1
				data[i].append(field_line.line.length)
				# if i==4:
				field_line.plot('100',c='magenta',alpha=0.5)
		simple_logger.info('counter: %s',counter)
	plt.axis('equal')
	plt.show()
if __name__=='__main__':
	main()
