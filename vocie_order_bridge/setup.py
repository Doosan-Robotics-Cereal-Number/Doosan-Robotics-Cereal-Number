from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vocie_order_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # voice_order 폴더의 모든 Python 파일 포함
        ('share/' + package_name + '/voice_order', glob('voice_order/*.py')),
        ('share/' + package_name + '/voice_order', glob('voice_order/*.txt')),
        ('share/' + package_name + '/voice_order', glob('voice_order/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leesh',
    maintainer_email='tkdgurdhkd12@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'voice_order_listener = vocie_order_bridge.voice_order_listener:main',
        ],
    },
)
