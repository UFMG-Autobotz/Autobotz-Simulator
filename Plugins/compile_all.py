import os
dirs = os.listdir('.')
for dir_ in dirs:
	if os.path.isdir(dir_):
		os.chdir(dir_)
		os.system('bash compile.sh')
		os.chdir('..')