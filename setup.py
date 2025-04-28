from setuptools import find_packages, setup

package_name = 'ros2daq'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dstups',
    maintainer_email='dstupski@uw.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'daqNode = ros2daq.daqNode:main',
        'analog_out_example = ros2daq.analog_out_example:main'
        ],
    },

)
