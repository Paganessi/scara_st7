from setuptools import find_packages, setup

package_name = 'scara_teleop'

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
    description='Comandos punto a punto del SCARA desde la terminal (goto, jog).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'goto = scara_teleop.goto:main',
            'jog = scara_teleop.jog:main',
        ],
    },
)
