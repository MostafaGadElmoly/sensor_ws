from setuptools import find_packages, setup

package_name = 'sensor_read'

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
    maintainer='mostafa',
    maintainer_email='Mostafa.gadelmoly@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publish_data = sensor_read.publish_data:main',
            'could_points = sensor_read.could_points:main',
            'multi_pub = sensor_read.multi_pub:main',
            'multi_points = sensor_read.multi_points:main',


        ],
    },
)
