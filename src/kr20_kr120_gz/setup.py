from setuptools import setup
from glob import glob
import os

package_name = 'kr20_kr120_gz'
share_dir = os.path.join('share', package_name)

setup(
    name=package_name,
    version='0.0.1',
    packages=[],  # no python modules, just assets/launch
    data_files=[
        # ament resource index entry so ROS 2 can "find" the package
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # package.xml goes under share/<pkg>
        (share_dir, ['package.xml']),
        # install assets
        (os.path.join(share_dir, 'launch'),  glob('launch/*.py')),
        (os.path.join(share_dir, 'worlds'),  glob('worlds/*.world')),
        (os.path.join(share_dir, 'models', 'kr20'),  glob('models/kr20/*')),
        (os.path.join(share_dir, 'models', 'kr120'), glob('models/kr120/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='',
    description='Gazebo Harmonic setup for KR20/KR120 cell',
    license='Apache-2.0',
)
