from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'gwonho_orchestrator'

def existing(pattern):
    files = glob(pattern)
    return files if files else []

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 루트 launch/ (현재 구조)
        ('share/' + package_name + '/launch', existing('launch/*')),
        # 패키지 내부 launch/ (혹시 나중에 옮겨도 동작)
        ('share/' + package_name + '/launch', existing(os.path.join(package_name, 'launch', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gwon_ho',
    maintainer_email='you@example.com',
    description='Orchestrator (action server/client) for Doosan e0509',
    license='BSD',
    # 🔥 py_modules 추가 - 패키지 내 모든 Python 모듈 명시
    py_modules=[
        'gwonho_orchestrator.gripper_drl_controller',  # 🔥 그리퍼 컨트롤러 추가!
    ],
    entry_points={
        'console_scripts': [
            'network_manager = gwonho_orchestrator.network_manager:main',
            'motion_control  = gwonho_orchestrator.motion_control:main',
            'network_manager_v2 = gwonho_orchestrator.network_manager_v2:main',  
            'motion_control_v2 = gwonho_orchestrator.motion_control_v2:main',    
            'network_manager_v3 = gwonho_orchestrator.network_manager_v3:main',  
            'motion_control_v3 = gwonho_orchestrator.motion_control_v3:main',
            'network_manager_v4 = gwonho_orchestrator.network_manager_v4:main',  
            'motion_control_v4 = gwonho_orchestrator.motion_control_v4:main',    
            'network_manager_v5 = gwonho_orchestrator.network_manager_v5:main',  
            'motion_control_v5 = gwonho_orchestrator.motion_control_v5:main',   
            
        ],
    },
)