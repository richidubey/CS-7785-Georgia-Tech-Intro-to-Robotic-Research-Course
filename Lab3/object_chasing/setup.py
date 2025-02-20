from setuptools import find_packages, setup

package_name = 'object_chasing'

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
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'select_object=object_chasing.select_object:main',
            'find_object=object_chasing.find_object:main',
            'get_object_range=object_chasing.get_object_range:main',
            'chase_object=object_chasing.chase_object:main',
        ],
    },
)
