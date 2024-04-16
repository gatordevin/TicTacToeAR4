from setuptools import find_packages, setup

package_name = 'tictacvis'

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
    maintainer='Thomas Le',
    maintainer_email='thomas.le@ufl.edu',
    description='Computer Vision node for Tic Tac Toe robot, Spring 2024',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'exec = tictacvis.TicTacToeVis:main',
        ],
    },
)
