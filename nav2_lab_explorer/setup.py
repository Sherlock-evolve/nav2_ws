from setuptools import setup


package_name = 'nav2_lab_explorer'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nav2_lab',
    maintainer_email='user@example.com',
    description='Frontier-based automatic exploration for nav2_lab mapping.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'explore_runner = nav2_lab_explorer.explore_runner:main',
        ],
    },
)
