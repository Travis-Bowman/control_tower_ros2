from setuptools import find_packages, setup

package_name = 'control_tower_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='tbowman@ltu.edu',
    description='Control Tower ROS 2 package',
    license='TODO: License declaration',
    tests_require=['pytest'],
)