import os
import numpy as np
import pandas as pd
import itertools
import collections
from Egistic_Nav_Env.envs import nav_env
import gym
from lgblkb_tools import logger,with_logging
import lgblkb_navigation.global_support as gsup

def main():
	env=gym.make('Egistic_Nav_Env:nav_env-v0')
	logger.debug('env.action_space:\n%s',env.action_space)
	# logger.debug('env.action_space:\n%s',env.env)
	logger.debug('env.metadata:\n%s',env.metadata)
	logger.debug('env.observation_space:\n%s',env.observation_space)
	logger.debug('env.reward_range:\n%s',env.reward_range)
	logger.debug('env.spec:\n%s',env.spec)
	logger.debug('env.action_space:\n%s',env.action_space)
	
	# gym.make('gym_foo:foo-v0')
	pass

if __name__=='__main__':
	main()
