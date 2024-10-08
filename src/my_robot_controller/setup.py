from setuptools import find_packages, setup

package_name = 'my_robot_controller'

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
    maintainer='ros2',
    maintainer_email='ros2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = my_robot_controller.my_first_node:main",
            "draw_circle = my_robot_controller.draw_circle:main",
            "pose_subscriber = my_robot_controller.pose_subscriber:main",
            "turtle_controller = my_robot_controller.turtle_controller:main",
            "face_tracker_node = my_robot_controller.face_tracker_node:main",
            "distance = my_robot_controller.distance:main",
            "motor_controller = my_robot_controller.motor_controller:main",
            "serial_test = my_robot_controller.serial_test:main",
            "keyboard_capture = my_robot_controller.keyboard_capture:main",
            "preset_node = my_robot_controller.preset_node:main",
            "camera_control = my_robot_controller.camera_control:main"
        ],
        
    },
)
