import os
import numpy as np
import pandas as pd
import itertools
import collections
from lgblkb_tools.global_support import *
from lgblkb_tools.log_support import with_logging,create_logger,simple_fmt_no_level
from lgblkb_tools.databases import sqla_orms,db_management as dbm
from box import Box
import socket

is_local_dev=os.environ.get('lgblkb') or socket.gethostname() in ['lgblkb-GT62VR-7RD']
simple_logger=create_logger('logs',log_format=simple_fmt_no_level)
project_folder=Folder(__file__)
configs=Box.from_yaml(filename=project_folder.get_filepath('configs_lgblkb.yaml' if is_local_dev else 'configs.yaml')).pg
mgr=dbm.Manager(configs.image_backend,host='94.247.135.91',username='docker',password='docker',port='8086')



def main():
	pass

if __name__=='__main__':
	main()
