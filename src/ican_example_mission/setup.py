from glob import glob
from setuptools import setup

package_name = "ican_example_mission"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="iCAN User",
    maintainer_email="user@example.com",
    description="Complete iCAN navigation and shooting mission package.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "one_target_mission_node = ican_example_mission.one_target_mission_node:main",
            "full_mission_node = ican_example_mission.full_mission_node:main",
        ],
    },
)
