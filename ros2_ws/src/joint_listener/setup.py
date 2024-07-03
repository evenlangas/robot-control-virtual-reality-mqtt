from setuptools import find_packages, setup

package_name = 'joint_listener'

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
    maintainer='node',
    maintainer_email='node@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "joint_subscriber = joint_listener.joint_subscriber:main",
            "joint_subscriber_mqtt = joint_listener.joint_subscriber_mqtt:main",
            "unity_subscriber = joint_listener.unity_subscriber:main",
            "unity_subscriber_mqtt = joint_listener.unity_subscriber_mqtt:main",
            "test_greie = joint_listener.test:main",
            "robotiq_2f_gripper_ctrl = joint_listener.gripper:main",
            "aruco_detector = joint_listener.arucotest:main",
            "aruco_detector_test = joint_listener.arucotest2:main",
            "Graph_Node = joint_listener.graphing:main"
        ],
    },
)
