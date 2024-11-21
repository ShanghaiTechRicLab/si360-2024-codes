from setuptools import find_packages, setup

package_name = 'kitti2rosbag'

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
    maintainer='dengqi',
    maintainer_email='dengqi935@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "lidar_converter = kitti2rosbag.lidar_converter:main",
            "all_converter = kitti2rosbag.all_converter:main",
        ],
    },
)
