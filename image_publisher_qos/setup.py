from setuptools import setup

package_name = 'image_publisher_qos'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'rclpy', 'cv_bridge', 'opencv-python'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='A simple ROS2 image publisher',
    license='License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher_node = image_publisher_qos.image_publisher_node:main',
        ],
    },
)

