import more_itertools as mi
import os
import numpy as np
import pandas as pd
import itertools
import collections
import ortools
import matplotlib.pyplot as plt
import scipy.spatial.distance as scispd
import scipy.spatial as scisp
from egistic_navigation.global_support import simple_logger,with_logging
import egistic_navigation.global_support as gsup
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

"""Simple travelling salesman problem between cities."""

class SimpleTSP(object):
	
	def __init__(self,city_coors):
		self.city_coors=city_coors
		self.distance_matrix=scisp.distance_matrix(self.city_coors,self.city_coors)
		self.manager=None
		self.routing=None
	
	def get_data_model(self,num_vehicles=1,depot=0):
		data=dict(distance_matrix=self.distance_matrix,num_vehicles=num_vehicles,depot=depot)
		return data
	
	def run(self,num_vehicles=1,depot=0,show=False):
		self.manager=pywrapcp.RoutingIndexManager(len(self.distance_matrix),num_vehicles,depot)
		self.routing=pywrapcp.RoutingModel(self.manager)
		transit_callback_index=self.routing.RegisterTransitCallback(self.distance_callback)
		
		# Define cost of each arc.
		self.routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
		# Setting first solution heuristic.
		search_parameters=pywrapcp.DefaultRoutingSearchParameters()
		search_parameters.first_solution_strategy=routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
		assignment=self.routing.SolveWithParameters(search_parameters)
		if assignment:
			return self.obtain_results(assignment,show=show)
		else:
			simple_logger.warning('No solution obtained.')
	
	def distance_callback(self,from_index,to_index):
		"""Returns the distance between the two nodes."""
		# Convert from routing variable Index to distance matrix NodeIndex.
		from_node=self.manager.IndexToNode(from_index)
		to_node=self.manager.IndexToNode(to_index)
		return self.distance_matrix[from_node][to_node]
	
	def obtain_results(self,assignment,show=False):
		"""Prints assignment on console."""
		# simple_logger.debug('Objective: {} meters'.format(assignment.ObjectiveValue()))
		index=self.routing.Start(0)
		plan_output='Route for vehicle 0:\n'
		route_distance=0
		nodes=list()
		while True:
			node=self.manager.IndexToNode(index)
			nodes.append(node)
			plan_output+=' {} ->'.format(node)
			previous_index=index
			if self.routing.IsEnd(index): break
			index=assignment.Value(self.routing.NextVar(index))
			route_distance+=self.routing.GetArcCostForVehicle(previous_index,index,0)
		# plan_output+=' {}\n'.format(manager.IndexToNode(index))
		# plan_output+='Route distance: {} meters\n'.format(route_distance)
		# simple_logger.debug('plan_output:\n%s',plan_output)
		# simple_logger.debug('nodes:\n%s',nodes)
		if show:
			plt.scatter(self.city_coors[:,0],self.city_coors[:,1])
			# plt.scatter(*depot_point)
			plt.plot(self.city_coors[nodes][:,0],self.city_coors[nodes][:,1])
			plt.show()
		return nodes

@with_logging()
def main():
	points=np.random.rand(100,2)*100
	stsp=SimpleTSP(points)
	route=stsp.run(depot=0,show=True)
	simple_logger.debug('points[route]:\n%s',points[route])

if __name__=='__main__':
	main()
