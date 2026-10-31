from setuptools import setup

package_name = 'el05_serial_tools'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    entry_points={'console_scripts': ['el05_serial_monitor = el05_serial_tools.serial_monitor:main']},
)
