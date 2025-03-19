from setuptools import find_packages, setup

package_name = 'test_package'

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
    maintainer='porterclev',
    maintainer_email='porterclev@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can_talk = test_package.can_test:main',
            'mini_joy = test_package.mini_joy:main',
            'mini_drive = test_package.mini_drive:main',
            'main_joy = test_package.main_joy:main',
            'main_drive = test_package.main_drive:main',
            'camera_pub = test_package.camera_pub:main',
            'camera_sub = test_package.camera_sub:main'
        ],
    },
)
