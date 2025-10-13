from setuptools import setup
from glob import glob
import os

package_name = 'kr20_kr120_bringup'
share_dir = os.path.join('share', package_name)

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share_dir, ['package.xml']),
        (os.path.join(share_dir, 'launch'),  glob('launch/*.py')),
        (os.path.join(share_dir, 'config'),  glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='',
    description='Bringup for KR20/KR120 cell',
    license='Apache-2.0',
)
