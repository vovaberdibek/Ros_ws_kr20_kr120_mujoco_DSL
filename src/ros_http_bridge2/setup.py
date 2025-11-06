from setuptools import setup, find_packages
pkg = 'ros_http_bridge2'

setup(
    name=pkg,
    version='0.0.1',
    packages=find_packages(include=[pkg, f"{pkg}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{pkg}']),
        (f'share/{pkg}', ['package.xml']),
        (f'share/{pkg}', [f'{pkg}/combined_dsl.lark']),
    ],
    install_requires=['setuptools','fastapi','uvicorn','pydantic','lark'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='HTTP ↔ ROS 2 bridge with DSL runner',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # map to your actual files/functions
            'ros_http_bridge = ros_http_bridge2.ros_http_bridge_rclpy:main',
            'confirm_server  = ros_http_bridge2.confirmation_service_rclpy:main',
            'dsl_main        = ros_http_bridge2.main:main',
            'mock_servers    = ros_http_bridge2.mock_servers:main',   
        ],
    },
)
