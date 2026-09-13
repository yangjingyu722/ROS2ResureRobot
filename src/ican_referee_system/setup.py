from glob import glob
import os
from setuptools import find_packages, setup


package_name = "ican_referee_system"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="iCAN Sim",
    maintainer_email="todo@example.com",
    description="Live referee scoring for the iCAN shooting mission.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["referee_node = ican_referee_system.referee_node:main"]},
)
