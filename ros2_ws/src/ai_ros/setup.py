from setuptools import find_packages, setup

package_name = 'ai_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cup_view.launch.py', 'launch/clean_workspace.launch.py']),
        ('share/' + package_name + '/rviz',   ['rviz/cup_view.rviz']),
    ],
    install_requires=[
        'setuptools',
        'websockets>=12.0,<16.0',
    ],
    zip_safe=True,
    maintainer='gwon_ho',
    maintainer_email='gwon_ho@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cup_detecting_node = ai_ros.cup_detecting_node:main',
            'cup_grab = ai_ros.cup_grab:main', 
            'cup_detecting_unified_node = ai_ros.cup_detection_unified_node:main',
            'cup_capture = ai_ros.cup_capture:main',
            'realtime_cup_detection_node = ai_ros.realtime_cup_detection_node:main',
            'cup_detection_gpt_node = ai_ros.cup_detection_gpt_node:main',
            'cup_detection_clip_node = ai_ros.cup_detection_clip_node:main',
            'realsense_node = ai_ros.realsense_node:main',
            'cleanup_orchestrator_node = ai_ros.cleanup_orchestrator_node:main',
        ],
    },
)
