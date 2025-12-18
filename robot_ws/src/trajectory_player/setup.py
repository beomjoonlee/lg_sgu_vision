from setuptools import find_packages, setup

package_name = 'trajectory_player'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/csv_moveit_player.launch.py']),
        ('share/' + package_name + '/launch',
         ['launch/topic_moveit_player.launch.py']),
    ],
    install_requires=['setuptools', 'moveit_configs_utils'],
    zip_safe=True,
    maintainer='lbj',
    maintainer_email='lbj@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # ros2 run trajectory_player csv_moveit_player
            #  → trajectory_player/csv_moveit_player.py 의 main() 실행
            'csv_moveit_player = trajectory_player.csv_moveit_player:main',
            'topic_moveit_player = trajectory_player.topic_moveit_player:main',
        ],
    },
)
