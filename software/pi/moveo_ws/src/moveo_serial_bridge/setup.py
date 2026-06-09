from setuptools import setup

package_name = 'moveo_serial_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='armpi',
    maintainer_email='armpi@todo.todo',
    description='Simple serial bridge for ESP32-S3',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge = moveo_serial_bridge.serial_bridge:main',
        ],
    },
)
