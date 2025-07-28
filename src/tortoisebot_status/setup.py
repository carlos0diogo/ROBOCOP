from setuptools import setup

package_name = 'tortoisebot_status'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/status.launch.py']),
        ('share/' + package_name + '/config', ['config/status_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TortoiseBot Dev',
    maintainer_email='dev@tortoisebot.com',
    description='System status monitoring for TortoiseBot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_monitor = tortoisebot_status.battery_monitor:main',
            'system_monitor = tortoisebot_status.system_monitor:main',
            'diagnostics_node = tortoisebot_status.diagnostics_node:main',
        ],
    },
)
