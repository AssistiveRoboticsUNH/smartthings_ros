from setuptools import setup

package_name = 'smartthings_ros'

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
    maintainer='pac48',
    maintainer_email='pac48@wildcats.unh.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'smartthings_node = smartthings_ros.smartthings_node_sensor:main',
            'smartthings_node_lab = smartthings_ros.smartthings_node_sensor_lab:main',
            'smartplug_node = smartthings_ros.smart_plug_sensor:main',
            'activity_node = smartthings_ros.activity_recognition_node:main',
	    'initial_pose = smartthings_ros.initial_position_node:main',
        ],
    },
)
