from setuptools import find_packages, setup
from glob import glob


package_name = 'autopilot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        ('lib/' + package_name, glob('nodes/*')),
        ('lib/' + package_name, glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='animated',
    maintainer_email='73669160+ChrisNassif@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autopilot = autopilot_node:main',
            'telemetry = telemetry_node:main'
        ],
    },
)
