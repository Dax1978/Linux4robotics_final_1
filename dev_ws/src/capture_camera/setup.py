from setuptools import find_packages, setup
import os

package_name = 'capture_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Подключаю launch файлы
        (os.path.join('share', package_name, 'launch'), ['launch/camera_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dax',
    maintainer_email='John-henos@yandex.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capture_camera = capture_camera.capture_camera:main'
        ],
    },
)
