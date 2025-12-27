from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'rgbd_yolo_3d'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minwoo',
    maintainer_email='kmw2622@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
             'yolo = rgbd_yolo_3d.yolo:main',
        ],
    },
)
