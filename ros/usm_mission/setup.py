from setuptools import find_packages, setup

package_name = 'usm_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Oguz Umut Meric',
    maintainer_email='oguzmeric9@gmail.com',
    description='Mission and offboard flight control for uav-semantic-mapping.',
    license='AGPL-3.0-only',
    entry_points={
        'console_scripts': [
            'offboard_hover = usm_mission.offboard_hover:main',
        ],
    },
)
