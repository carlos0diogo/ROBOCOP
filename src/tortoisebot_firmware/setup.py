from setuptools import find_packages, setup

package_name = 'tortoisebot_firmware'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seu Nome',
    maintainer_email='seu-email@exemplo.com',
    description='Controle de hardware para TortoiseBot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'differential_drive = tortoisebot_firmware.differential_drive:main',
            'ultrasonic_sensor = tortoisebot_firmware.ultrasonic_sensor:main',
        ],
    },
)
