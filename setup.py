from setuptools import setup

package_name = 'mk_0'

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
    maintainer='kodogyu',
    maintainer_email='kodogyu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_processor = mk_0.image_processor:main',
            'teleop_mk_0 = mk_0.teleop_mk_0:main',
            'position_controller = mk_0.position_controller:main',
        ],
    },
)
