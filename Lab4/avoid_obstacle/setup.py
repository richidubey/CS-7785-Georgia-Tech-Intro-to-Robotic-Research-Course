from setuptools import find_packages, setup

package_name = 'avoid_obstacle'

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
    maintainer='richi',
    maintainer_email='richidubey@gmail.com',
    description='Package Lab4 ',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'actual_odom=avoid_obstacle.actual_odom:main',
            'state_manager=avoid_obstacle.state_manager:main',
            'goto_goal=avoid_obstacle.goto_goal:main',
            'avoid_obstacles=avoid_obstacle.avoid_obstacles:main'
        ],
    },
)
