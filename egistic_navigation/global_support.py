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

pd.set_option('display.max_colwidth',-1)
pd.set_option('display.max_rows',500)
pd.set_option('display.max_columns',500)
pd.set_option('display.width',1000)
pd.options.mode.chained_assignment=None


is_local_dev=os.environ.get('lgblkb') or socket.gethostname() in ['lgblkb-GT62VR-7RD']
simple_logger=create_logger('logs',log_format=simple_fmt_no_level)
project_folder=Folder(__file__)
configs=Box.from_yaml(filename=project_folder.get_filepath('configs_local.yaml' if is_local_dev else 'configs.yaml')).pg
pg_configs_default=configs.image_backend
if is_local_dev:
	pg_configs_default=pg_configs_default.local

# get_manager=partial(dbm.Manager,pg_configs_default)
mgr=dbm.Manager(pg_configs_default)
# mgr=dbm.Manager(configs.image_backend,host='94.247.135.91',username='docker',password='docker',port='8086')



def main():
	pass

if __name__=='__main__':
	main()
