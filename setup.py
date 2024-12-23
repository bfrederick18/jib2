from setuptools import find_packages, setup
from glob import glob

package_name = 'jib2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brandon',
    maintainer_email='brandon@caltech.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pirouetteandwave = jib2.pirouetteandwave:main',
            'squat = jib2.squat:main',
            'balldrop = jib2.balldrop:main',
            'ballinhand = jib2.ballinhand:main',
            'squatdrop = jib2.squatdrop:main',
            'throw = jib2.throw:main',
            'cannonball = jib2.cannonball:main',
            'subscriber = jib2.subscriber:main'
        ],
    },
)
