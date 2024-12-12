from setuptools import find_packages, setup

package_name = 'turtlebot_ekf'

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
    maintainer='sadeep',
    maintainer_email='sadeepm20@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ekf_node = turtlebot_ekf.ekf:main',
            'visualizer_node = turtlebot_ekf.visualizer:main'
        ],
    },
)
