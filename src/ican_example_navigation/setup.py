from glob import glob
from setuptools import setup

package_name = "ican_example_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/maps", glob("maps/*")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="iCAN User",
    maintainer_email="user@example.com",
    description="Example Nav2 bringup package for the iCAN practice simulation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "initial_pose_publisher = ican_example_navigation.initial_pose_publisher:main",
        ],
    },
)
