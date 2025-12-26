from setuptools import find_packages, setup

package_name = 'practice'

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
    maintainer='ssafy',
    maintainer_email='inhyukjeong10@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            '4008 = practice.4008:main',
            '4007 = practice.4007:main',
            'yolov5 = practice.yolov5:main',
            '4014_1 = practice.4014_1:main',
            '4014_2 = practice.4014_2:main',
            '4014_3 = practice.4014_3:main',
        ],
    },
)
