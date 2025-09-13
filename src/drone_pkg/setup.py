from setuptools import find_packages, setup

package_name = 'drone_pkg'

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
    maintainer='spiderweb',
    maintainer_email='matt.vanwyk2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_state_machine = drone_pkg.drone_state_machine:main',
            'mavsdk_node = drone_pkg.mavsdk_node:main'
        ],
    },
)
