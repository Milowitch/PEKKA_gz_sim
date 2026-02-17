import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover'

script_dir = os.path.join(os.path.dirname(__file__), 'script')
entry_points = [
    f"{os.path.splitext(os.path.basename(script))[0]} = script.{os.path.splitext(os.path.basename(script))[0]}:main"
    for script in glob(os.path.join(script_dir, "*.py"))
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[y][a]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mekkkk',
    maintainer_email='mekkkk@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': entry_points,
    },
)
