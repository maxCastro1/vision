from setuptools import find_packages, setup

package_name = 'aisd_motion'

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
    maintainer='maxime',
    maintainer_email='maxmizlg512@gmail.com',
    description='packages about moving the turtle',
    license='no license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
'move = aisd_motion.move:main',
        ],
    },
)
