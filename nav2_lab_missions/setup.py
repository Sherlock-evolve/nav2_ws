from glob import glob
from setuptools import setup

package_name = 'nav2_lab_missions'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nav2_lab',
    maintainer_email='user@example.com',
    description='Mission runner and logging nodes for the Nav2 simulation lab.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mission_runner = nav2_lab_missions.mission_runner:main',
            'mission_logger = nav2_lab_missions.mission_logger:main',
        ],
    },
)
