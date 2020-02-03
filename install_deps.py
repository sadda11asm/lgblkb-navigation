from lgblkb_tools import logger,Folder
from lgblkb_tools.common.utils import run_cmd

def main():
	deps=[
	      'matplotlib',
	      'more_itertools',
	      'networkx',
	      'numpy',
	      'ortools',
	      'pandas',
	      'scipy',
	      'Shapely',
	      'scikit_learn',
	      'sortedcontainers',
	      'typing',
	      'visvalingamwyatt',]
	for dep in deps:
		run_cmd(f'pipenv install {dep}')
	
	pass

if __name__=='__main__':
	main()
