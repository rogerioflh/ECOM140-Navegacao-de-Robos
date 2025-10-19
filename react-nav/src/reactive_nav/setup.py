from setuptools import find_packages, setup

package_name = 'reactive_nav'

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
    maintainer='rogers',
    maintainer_email='rogeriofilho63@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'bug2_node = reactive_nav.bug2_controller:main', 'tangent_bug_node = reactive_nav.tangent_bug_controller:main',
        ],
    },
)
