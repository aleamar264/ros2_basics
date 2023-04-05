from setuptools import setup

package_name = 'ros2_motion_python'

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
    maintainer='aleamar264',
    maintainer_email='alejandroamar66@gmail.com',
    description='ROS2 simple motion',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_simple_motion=ros2_motion_python.simple_turtlesim_motion:main',
            'cleaner=ros2_motion_python.clean:main'

        ],
    },
)
