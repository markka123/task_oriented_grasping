import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'peg_in_hole_task'


def package_files(directory):
    """Recursively collect non-Python data files for installation."""
    paths = []
    for (path, _, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),  glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),  glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'),  glob('worlds/*.world')),
    ] + [
        (os.path.join('share', package_name, os.path.dirname(f)), [f])
        for f in package_files('models')
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Markus',
    maintainer_email='markus.gk@outlook.com',
    description='Peg-in-hole task: scene spawning and parametrised grasp-and-insert pipeline.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'spawn_task_objects = peg_in_hole_task.spawn_task_objects:main',
            'grasp_and_insert   = peg_in_hole_task.grasp_and_insert:main',
        ],
    },
)
