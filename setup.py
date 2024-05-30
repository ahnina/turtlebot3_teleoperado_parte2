from setuptools import find_packages, setup

package_name = 'turtlebot3_teleop2'

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
    maintainer='ana',
    maintainer_email='ana.marques@sou.inteli.edu.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
       'console_scripts': [
            'teleop = turtlebot3_teleo2p.teleop:main',     
            'stop_robot_client = turtlebot3_teleop2.stop_robot_client:main',   
            'webcan_publisher = turtlebot3_teleop2.sender:main'     
        ],
    },
)
