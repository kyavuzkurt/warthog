from setuptools import find_packages, setup

package_name = 'warthog_controller'

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
    maintainer='K. Yavuz Kurt',
    maintainer_email='k.yavuzkurt1@gmail.com',
    description='Controller package for Warthog robot',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cmd_pub = warthog_controller.cmd_pub:main',
        ],
    },
)
