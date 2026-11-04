from glob import glob

from setuptools import find_packages, setup

package_name = 'usm_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Oguz Umut Meric',
    maintainer_email='oguzmeric9@gmail.com',
    description='Live object detection for uav-semantic-mapping.',
    license='AGPL-3.0-only',
    entry_points={
        'console_scripts': [
            'live_detector = usm_detection.live_detector:main',
        ],
    },
)
