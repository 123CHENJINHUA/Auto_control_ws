from setuptools import setup

package_name = 'button_command_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='unknown@example.com',
    description='Bridge VR controller buttons to tool commands over TCP.',
    license='MIT',
    tests_require=[],
    entry_points={
        'console_scripts': [
            'button_command_bridge_node = button_command_bridge.node:main',
        ],
    },
)
