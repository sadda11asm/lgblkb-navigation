import os
# from lgblkb_tools import sqla_orms,db_management as dbm
import socket

import pandas as pd
from lgblkb_tools import TheLogger,Folder

pd.set_option('display.max_colwidth',-1)
pd.set_option('display.max_rows',500)
pd.set_option('display.max_columns',500)
pd.set_option('display.width',1000)
pd.options.mode.chained_assignment=None

project_folder=Folder(__file__)
logs_folder=project_folder['logs']

is_local_dev=os.environ.get('lgblkb') or socket.gethostname() in ['lgblkb-GT62VR-7RD']
logger=TheLogger('lgblkb_logger')

# configs=Box.from_yaml(filename=project_folder.get_filepath('configs_local.yaml' if is_local_dev else 'configs.yaml')).pg
# pg_configs_default=configs.image_backend
# if is_local_dev:
# 	pg_configs_default=pg_configs_default.local

# get_manager=partial(dbm.Manager,pg_configs_default)
# mgr=dbm.Manager(pg_configs_default)
# mgr=dbm.Manager(configs.image_backend,host='94.247.135.91',username='docker',password='docker',port='8086')


def main():
	pass

if __name__=='__main__':
	main()
