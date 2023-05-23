from setuptools import setup

package_name = 'uav_testing_scripts'

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
    maintainer='kw',
    maintainer_email='kianweetan2002@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'controller = uav_testing_scripts.controller:main',
        	'talker = uav_testing_scripts.publisher_member_function:main',
        ],
    },
)
