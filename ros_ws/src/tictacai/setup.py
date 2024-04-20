from setuptools import find_packages, setup
import os
import glob

package_name = 'tictacai'

pthlist = [(os.path.join('share', package_name, os.path.split(path)[0]), [path]) for path in glob.glob('resource/**', recursive=True)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        "tictacai": ["*.dat"]
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kubuntu',
    maintainer_email='devon.limcangco@ufl.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'exec = tictacai.main:main',
        ],
    },
)
