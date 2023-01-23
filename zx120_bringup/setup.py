from setuptools import setup

package_name = 'zx120_bringup'

setup(
    name=package_name,
    version='0.15.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Daisuke Endo',
    author_email='endou-d177cl@pwri.go.jp',
    maintainer='Genki Yamauchi',
    maintainer_email='yamauchi-g573bs@pwri.go.jp',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Translate PoseStamped type message from GNSS to odom type message.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_to_odom = zx120_bringup.poseStamped2Odometry:main',
        ],
    },
)