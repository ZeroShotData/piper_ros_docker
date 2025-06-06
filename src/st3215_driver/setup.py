from setuptools import setup

package_name = 'st3215_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Laz',
    maintainer_email='laz@zeroshotdata.com',
    description='Raw-tick driver for ST3215',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'st3215_servo = st3215_driver.servo_node:main',
        ],
    },
)