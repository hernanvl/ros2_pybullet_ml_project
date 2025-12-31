from setuptools import setup

package_name = 'pybullet_sim'

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
    maintainer='hernan',
    maintainer_email='example@example.com',
    description='PyBullet simulation node for ROS2 Jazzy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pybullet_node = pybullet_sim.pybullet_node:main',
        ],
    },
)
