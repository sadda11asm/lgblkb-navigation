import subprocess

def run_cmd(commands,**kwargs):
	if isinstance(commands,str):
		steps=[commands]
	else:
		steps=commands
	for step in steps:
		print('step: ',step,'\n############################################')
		subprocess.run(step,**dict(dict(check=True,shell=True),**kwargs))

def main():
	steps=list()
	# steps.append('pytest .')
	steps.append('rm -r dist build || true ')
	# steps.append('pipenv lock')
	steps.append('pipenv lock -r > requirements.txt')
	steps.append('git commit -am "Updated requirements.txt" || true')
	steps.append('bumpversion patch')
	steps.append('pipenv run python setup.py bdist_wheel')
	steps.append('git commit -am "Updated ChangeLog" || true')
	steps.append('twine upload dist/* || true')
	steps.append('pip install --no-cache-dir lgblkb-navigation -U')
	steps.append('pip install --no-cache-dir lgblkb-navigation -U')
	# steps.append('git push')
	
	run_cmd(steps)
	

if __name__=='__main__':
	main()
