from setuptools import find_packages, setup

package_name = 'local_pkg'

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
    maintainer='rajwardhan',
    maintainer_email='rajwardhan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "serial_node_rotor = local_pkg.Serial_node_rotor:main",
            "serial_node_imu = local_pkg.Serial_node_imu:main",
        ],
    },
)
