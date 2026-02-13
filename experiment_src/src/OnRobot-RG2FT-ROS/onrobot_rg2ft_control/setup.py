from setuptools import setup, find_packages

package_name = 'onrobot_rg2ft_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    install_requires=['setuptools', 'pymodbus'],
    zip_safe=True,
    maintainer='Ian Chuang',
    maintainer_email='itchuang@ucdavis.edu',
    description='ROS 2 control node for the OnRobot RG2-FT gripper',
    license='MIT',
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rg2ft_control.launch.py']),
    ],
    entry_points={
        'console_scripts': [
            'rg2ft_driver = onrobot_rg2ft_control.OnRobotRG2FTDriver:main',
        ],
    },
)