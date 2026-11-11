from setuptools import find_packages, setup

package_name = 'mi_mecanum_control'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manuel2404',
    maintainer_email='manuel.ortiz@eia.edu.co',
    description='Cinemática (Q, W, J) y navegación con A* para mi_robot_mecanum',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cinematica = mi_mecanum_control.cinematica:main',
            'navegar_astar = mi_mecanum_control.navegar_astar:main',
            'probar_offline = mi_mecanum_control.probar_offline:main',
        ],
    },
)
