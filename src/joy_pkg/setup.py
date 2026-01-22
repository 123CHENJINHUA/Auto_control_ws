from setuptools import find_packages, setup

package_name = 'joy_pkg'

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
    maintainer='hkust',
    maintainer_email='jchenjb@connect.ust.hk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_trans = joy_pkg.joy_trans:main',
            'joy_sub = joy_pkg.joy_sub:main',
            'joy2robot = joy_pkg.joy2robot:main',
        ],
    },
)
