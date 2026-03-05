from setuptools import find_packages, setup

package_name = 'roo_libcamera'

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
    maintainer='roo',
    maintainer_email='roo@todo.todo',
    description='Libcamera-native camera node (bypasses V4L2 compat)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'libcamera_node = roo_libcamera.libcamera_node:main',
            'compressed_republisher = roo_libcamera.compressed_republisher:main',
        ],
    },
)
