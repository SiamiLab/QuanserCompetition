from setuptools import setup

package_name = 'pack_qcar'
submodules = f'{package_name}/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'qcar_node = {package_name}.qcar_node:main',
            f'gps_node = {package_name}.gps_node:main',
            f'lidar_node = {package_name}.lidar_node:main',
            f'rgbd_node = {package_name}.rgbd_node:main',
            f'imgviewer_node = {package_name}.imgviewer_node:main',
            f'odom_node = {package_name}.odom_node:main',
            f'pose_node = {package_name}.pose_node:main',
            f'waypoint_node = {package_name}.waypoint_node:main',
            f'objectposition_node = {package_name}.objectposition_node:main',
            f'gamepad_node = {package_name}.gamepad_node:main',
        ],
    },
)