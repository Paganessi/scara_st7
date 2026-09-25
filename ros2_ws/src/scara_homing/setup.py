from setuptools import find_packages, setup

package_name = 'scara_homing'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matteo (Grupo 6-F)',
    maintainer_email='amavaga@gmail.com',
    description='Rutina de homing del SCARA (maquina de estados, servicio /scara/home).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'homing_node = scara_homing.homing_node:main',
        ],
    },
)
