from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'project2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name, 'launch'), glob('launch/*.py')),
        # Rviz:
        ('share/' + package_name + '/rviz', 
        glob('rviz/*.rviz')),
        # SDF:
        ('share/' + package_name + '/models/ur5_rg2', 
        glob('models/ur5_rg2/*.sdf')),
        # Visual meshes:
        ('share/' + package_name + '/models/ur5_rg2/meshes/visual/ur5', 
            glob('models/ur5_rg2/meshes/visual/ur5/*.dae')),
        ('share/' + package_name + '/models/ur5_rg2/meshes/visual/rg2', 
            glob('models/ur5_rg2/meshes/visual/rg2/*.dae')),
        # Colision meshes:
        ('share/' + package_name + '/models/ur5_rg2/meshes/collision/ur5', 
            glob('models/ur5_rg2/meshes/collision/ur5/*.stl')),
        ('share/' + package_name + '/models/ur5_rg2/meshes/collision/rg2', 
            glob('models/ur5_rg2/meshes/collision/rg2/*.stl')),        
        # URDF:
        ('share/' + package_name + '/models/ur5_rg2', 
            glob('models/ur5_rg2/*.urdf')),
        # YAML:
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # Cube sdf model:
        ('share/' + package_name + '/models/cube', glob('models/cube/*.sdf'))
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jan',
    maintainer_email='jan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_to_gazebo = project2.GUI_to_Gazebo:main'
        ],
    },
)
