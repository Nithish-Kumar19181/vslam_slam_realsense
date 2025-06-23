from setuptools import find_packages, setup

package_name = 'realsense_imu_fusion'

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
    maintainer='nithish',
    maintainer_email='nith600e@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'realsense_imu_fusion_node = realsense_imu_fusion.realsense_imu_fusion_node:main',
        ],
    },

)