from setuptools import find_packages, setup

package_name = 'scara_tools'

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
    description='Herramientas de puesta en marcha: repetibilidad del homing, escalon PID, zona muerta.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'repeat_home = scara_tools.repeat_home:main',
            'step_test = scara_tools.step_test:main',
            'deadzone_test = scara_tools.deadzone_test:main',
            'fake_esp32 = scara_tools.fake_esp32:main',
        ],
    },
)
