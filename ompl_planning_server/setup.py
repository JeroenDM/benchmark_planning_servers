# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['ompl_planning_server', 'ompl_planning_server.robot',
              'ompl_planning_server.servers'],
    package_dir={'': 'src'},
)

setup(**setup_args)
