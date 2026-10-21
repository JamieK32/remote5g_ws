from glob import glob  #glob通过通配符寻找文件

from setuptools import find_packages, setup #自动寻找python包


setup(
    name='remote5g_autonomy', #Python安装发行包的名称
    version='0.1.0', #软件版本号
    packages=find_packages(exclude=['test']), #不把名为test的作为应用包安装
    data_files=[ #安装非Python文件
        ('share/ament_index/resource_index/packages', ['resource/remote5g_autonomy']),
        ('share/remote5g_autonomy', ['package.xml', 'README.md', 'LICENSE']),
        ('share/remote5g_autonomy/config', glob('config/*.yaml')), #作用是把 config/ 目录下匹配的 YAML 文件安装到软件包的共享目录中。
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
