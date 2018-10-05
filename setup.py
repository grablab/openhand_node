#!/usr/env python 

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d1 = generate_distutils_setup(
	packages=['openhand_node'],
	package_dir={'': 'src'}
)

setup(**d1)
