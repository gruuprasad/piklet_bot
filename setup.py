from setuptools import setup
import os
from glob import glob

package_name = 'piklet_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf') + glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'urdf/meshes'), glob('urdf/meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Guruprasad Hegde',
    maintainer_email='gruuprasada@gmail.com',
    description='This package contains the robot description for the Piklet Bot.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_joint_publisher = piklet_description.wheel_joint_publisher:main',
        ],
    },
)
