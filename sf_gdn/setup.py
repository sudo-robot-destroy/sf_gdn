from setuptools import setup
from glob import glob
import os


package_name = 'sf_gdn'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='otooleat',
    maintainer_email='otooleat@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #      ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    #     (os.path.join('share', package_name, 'launch'), glob('launch/*.xml')),
    # ],
    
    entry_points={
        'console_scripts': [
                'talker = sf_gdn.publisher_member_function:main',
                'factor_graph = sf_gdn.factor_graph:main',
                'bearing_pub = sf_gdn.bearing_publisher:main',
                'odometry_ser = sf_gdn.odometry_service:main'
        ],
    },
)
