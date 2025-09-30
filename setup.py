from setuptools import find_packages, setup

package_name = 'follow_the_gap_charlie'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', [
        'launch/follow_the_gap_charlie.launch.py',
    ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='clip2004',
    maintainer_email='felipe.mercado59@eia.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gap_finder_charlie = follow_the_gap_charlie.gap_finder_charlie:main',
            'gap_controller_charlie = follow_the_gap_charlie.gap_controller_charlie:main',
        ],
    },
)
