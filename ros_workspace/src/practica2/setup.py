from setuptools import find_packages, setup

package_name = 'practica2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Katherine Lorenzo',
    maintainer_email='katherine.lorenzo@opendeusto.es',
    description='Práctica 2: ingesta, calibración y procesamiento de imágenes con ROS 2 y OpenCV.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'capturador_imagenes = practica2.capturador_imagenes:main',
            'aplicador_calibracion = practica2.aplicador_calibracion:main',
        ],
    },
)
