#!/usr/bin/env python

from setuptools import setup

setup(
	setup_requires=['pbr'],
	pbr=True,
)

def old_setup():
	import setuptools
	# import wrapt
	# from box import Box
	from lgblkb_tools import Folder
	
	# with open("README.md","r") as fh:
	# 	long_description=fh.read()
	
	# root_folder=Path(__file__).parent
	root_folder=Folder(__file__,reactive=False)
	
	install_requires=[
		'pandas',
		'numpy',
		'python-box',
		'more-itertools',
		'sklearn',
		'scikit-image',
		'scikit_learn',
		'python_dateutil',
		'ruamel.yaml',
		'python_log_indenter',
		'colorlog',
		'requests',
		'docopt',
		'python-dotenv',
		'wrapt',
		
		'shapely',
		'pyproj',
		'geojson',
		'geopandas',
		'geojsonio',
		
		'matplotlib',
		
		'redis',
		'celery',
		'celery-singleton',
		'docker',
		'python-telegram-bot',
		'sqlalchemy',
		'geoalchemy2',
		'psycopg2',
		'jsonpickle',
		'fabric',
		# 'circus',
		'checksumdir',
		'joblib',
	
	]
	
	def setup(version):
		setuptools.setup(
			name="lgblkb_tools",
			version=version,
			author="Dias Bakhtiyarov",
			author_email="dbakhtiyarov@nu.edu.kz",
			description="Some useful tools for everyday routine coding improvisation)",
			long_description='',
			long_description_content_type="text/markdown",
			url="https://github.com/lgblkb/lgblkb_tools",
			packages=setuptools.find_packages(),
			classifiers=(
				"Programming Language :: Python :: 3.6",
				"License :: OSI Approved :: MIT License",
				"Operating System :: OS Independent",
			),
			install_requires=install_requires
		)
	
	# noinspection PyAbstractClass
	# class BoxProxy(wrapt.ObjectProxy):
	# 	def __init__(self,wrapped):
	# 		super(BoxProxy,self).__init__(wrapped=wrapped)
	# 		self._self_new_version=None
	# 		self._self_filepath=None
	# 		pass
	#
	# 	def elevate(self,major,minor,maintenance=None,tag=''):
	# 		print('Current version: ',self.version)
	# 		# logger.debug('current_version: %s',self.version)
	# 		version_portions,old_tag=self.get_version_portions_and_tag()
	# 		prev_major,prev_minor,prev_maint=version_portions
	# 		if major!=prev_major or minor!=prev_minor: maintenance=maintenance or 0
	# 		elif maintenance is None: maintenance=version_portions[-1]+1
	# 		new_version=f"{major}.{minor}.{maintenance}"
	# 		if tag: new_version+=f'.{tag}'
	# 		# logger.debug('new_version: %s',new_version)
	# 		print('new_version: ',new_version)
	#
	# 		self._self_new_version=new_version
	# 		return new_version
	#
	# 	def get_version_portions_and_tag(self):
	# 		portions=self.version.split('.',maxsplit=3)
	# 		if len(portions)==4: num_version,tag=portions[:3],portions[-1]
	# 		elif len(portions)==3: num_version,tag=portions,''
	# 		else: raise NotImplementedError('Version format is undefined.',dict(portions=portions))
	# 		num_portions=[int(x) for x in num_version]
	# 		return num_portions,tag
	#
	# 	def from_yaml(self,filepath,**kwargs):
	# 		data=self.__wrapped__.from_yaml(filename=filepath,**kwargs)
	# 		self._self_filepath=filepath
	# 		out=BoxProxy(data)
	# 		out._self_filepath=filepath
	# 		out._self_new_version=data.version
	# 		return out
	#
	# 	def update(self):
	# 		self.__wrapped__.version=self._self_new_version
	# 		self.to_yaml(filename=self._self_filepath)
	# 		# root_folder[os.path.split(self.filepath)[-1]]=dict(version=self.new_version)
	# 		return self
	
	# Elevator=BoxProxy(Box)
	
	# def get_package_info():
	# 	return Elevator.from_yaml(root_folder['package_info.yaml'])
	
	def main():
		build_folder=root_folder['build']
		dist_folder=root_folder['dist']
		# info=get_package_info()
		build_folder.delete()
		dist_folder.delete()
		# new_version=info.elevate(major=1,minor=0)
		# setup(new_version)
		# info.update()
		pass
	
	if __name__=='__main__':
		main()
		pass
