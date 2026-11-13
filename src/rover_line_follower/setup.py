from setuptools import find_packages, setup

package_name = 'rover_line_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ⬇⬇⬇ launch 파일도 설치되도록 추가
        ('share/' + package_name + '/launch', ['launch/line_follower.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Line follower node for my_rover using /camera/image_raw and /cmd_vel.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # ⬇⬇⬇ 실행 이름 = 패키지.모듈:함수
            'line_follower = rover_line_follower.line_follower:main',
            'image_processing = rover_line_follower.image_processing',
            #'visualizer_node = rover_line_follower.visualizer_node:main',
        ],
    },
)
