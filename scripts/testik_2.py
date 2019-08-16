import os
import numpy as np
import pandas as pd
import itertools
import collections
import shapely.geometry as shg
from egistic_navigation.global_support import simple_logger
from lgblkb_tools.folder_utils import create_iterated_path
from egistic_navigation import global_support as gsup
def test_create_iterated_path():
	path=r'/home/lgblkb/PycharmProjects/egistic_navigation/egistic_navigation/configs_local_koko.yaml'
	new_path=create_iterated_path(path)
	simple_logger.debug('new_path:\n%s',new_path)
	
	path=r'/home/lgblkb/PycharmProjects/egistic_navigation/egistic_navigation/configs_local_koko_2.yaml'
	new_path=create_iterated_path(path)
	simple_logger.debug('new_path:\n%s',new_path)
	
	path=r'/home/lgblkb/PycharmProjects/egistic_navigation/egistic_navigation/configs.yaml'
	new_path=create_iterated_path(path)
	simple_logger.debug('new_path:\n%s',new_path)

def main():
	test_folder=gsup.Folder(r'/home/lgblkb/PycharmProjects/egistic_navigation/egistic_navigation/test_folderchik_3')
	a=test_folder.create_iterated()
	simple_logger.debug('a: %s',a)
	
	
	
	pass

if __name__=='__main__':
	main()
