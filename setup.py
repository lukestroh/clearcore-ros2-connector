from setuptools import find_packages, setup

package_name = 'clearcore_udp'

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
    maintainer='Luke Strohbehn',
    maintainer_email='luke.strohbehn@gmail.com',
    description='A ROS2 communication method using a raw UDP Python socket to communicate with the Teknic ClearCore motor controller.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'pos_pub = {package_name}.udp_pos_publisher:main',
            f'target_pub = {package_name}.udp_target_publisher:main'
        ],
    },
)
