from setuptools import find_packages, setup
import os
from glob import glob

package_name = "peg_in_hole_task"


def package_files(directory):
    """Recursively collect non-python files for installation."""
    paths = []
    for (path, _, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ]
    + [
        (os.path.join("share", package_name, os.path.dirname(f)), [f])
        for f in package_files("models")
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="markus",
    maintainer_email="markus@todo.todo",
    description="Peg-in-hole task assets + spawner for UR Gazebo sim.",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "spawn_task_objects = peg_in_hole_task.spawn_task_objects:main",
        ],
    },
)
