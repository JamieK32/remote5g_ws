from glob import glob

from setuptools import find_packages, setup


setup(
    name='remote5g_autonomy',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/remote5g_autonomy']),
        ('share/remote5g_autonomy', ['package.xml', 'README.md', 'LICENSE']),
        ('share/remote5g_autonomy/config', glob('config/*.yaml')),
        ('share/remote5g_autonomy/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='JamieK',
    maintainer_email='jamiek@example.com',
    description='Visual lane perception, control and simulation tooling',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'lane_follow = remote5g_autonomy.app:main',
        'check_curve_follow = remote5g_autonomy.tools.check_curve_follow:main',
    ]},
)
