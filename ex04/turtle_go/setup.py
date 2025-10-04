from setuptools import find_packages, setup

package_name = 'turtle_go'

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
    maintainer='asanaaa',
    maintainer_email='a.chernysheva1@g.nsu.ru',
    description='Move to the given coordinates',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'go_to = turtle_go.move_to_goal:main'
        ],
    },
)
