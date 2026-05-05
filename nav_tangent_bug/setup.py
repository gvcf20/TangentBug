from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav_tangent_bug'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel',
    maintainer_email='seu@email.com',
    description='Exercício 1: Tangent Bug',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tangent_bug = nav_tangent_bug.tangent_bug_node:main',
            'tangent_bug_client = nav_tangent_bug.client_node:main',
        ],
    },
)