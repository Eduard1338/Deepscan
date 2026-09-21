from setuptools import find_packages, setup

package_name = 'deepscan_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/display.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/boat.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eduard',
    maintainer_email='eduard@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
