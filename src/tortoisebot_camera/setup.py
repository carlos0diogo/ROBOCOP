from setuptools import setup

package_name = 'tortoisebot_camera'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camera.launch.py']),
        ('share/' + package_name + '/config', ['config/camera_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TortoiseBot Dev',
    maintainer_email='dev@tortoisebot.com',
    description='Camera package for TortoiseBot - Webcam Lenovo 300 integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = tortoisebot_camera.camera_node:main',
            'camera_calibration = tortoisebot_camera.camera_calibration:main',
        ],
    },
)
