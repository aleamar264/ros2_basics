from setuptools import setup

package_name = 'udemy_examples'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
    'minimal_talker = udemy_examples.01_pubsub.talker:main',
    'minimal_listener = udemy_examples.01_pubsub.listener:main',
    'add_two_ints_server=udemy_examples.services.add_two_ints_server:main',
    'add_two_ints_client=udemy_examples.services.add_two_ints_client:main'
        ],
    },
)
