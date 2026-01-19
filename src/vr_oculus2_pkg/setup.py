from setuptools import find_packages, setup

package_name = 'vr_oculus2_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pure-python-adb'],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='user@example.com',
    description='ROS 2 node named VR publishing VR controller actions from vr_oculus2.',
    license='Proprietary',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'VR = vr_oculus2_pkg.node:main',
        ],
    },
)
