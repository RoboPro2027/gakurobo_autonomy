from setuptools import find_packages, setup

package_name = 'gakurobo_motion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/primitives.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoboPro2027',
    maintainer_email='robopro2027@example.invalid',
    description='Motion server for robot motion primitives.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': ['motion_server = gakurobo_motion.motion_server:main'],
    },
)
