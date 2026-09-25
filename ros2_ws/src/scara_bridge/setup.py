from setuptools import find_packages, setup

package_name = 'scara_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Matteo (Grupo 6-F)',
    maintainer_email='amavaga@gmail.com',
    description=(
        'Puente ROS 2 - ESP32 del SCARA: conversiones, limites, mux de comandos y keepalive.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bridge_node = scara_bridge.bridge_node:main',
        ],
    },
)
