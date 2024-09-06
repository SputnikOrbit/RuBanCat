from setuptools import setup

package_name = 'video_publisher'

setup(
name=package_name,
version='0.0.0',
packages=[package_name],
data_files=[
('share/ament_index/resource_index/packages',
['resource/' + package_name]),
('share/' + package_name, ['package.xml']),
],
install_requires=['setuptools'],
zip_safe=True,
maintainer='your_name',
maintainer_email='your_email@example.com',
description='ROS 2 package to publish video frames using OpenCV',
license='Apache License 2.0',
tests_require=['pytest'],
entry_points={
'console_scripts': [
'video_publisher_node = video_publisher.video_publisher_node:main',
],
},
)