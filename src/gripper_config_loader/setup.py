from setuptools import setup, find_packages

package_name = 'gripper_config_loader'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    install_requires=['setuptools', 'rclpy', 'PyYAML'],
    zip_safe=True,
    maintainer='JulienRineau',
    maintainer_email='julien.rineau@berkeley.edu',
    description='Loader node that validates a gripper YAML configuration and publishes it as ROS parameters.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gripper_config_loader = gripper_config_loader.loader_node:main',
        ],
    },
) 