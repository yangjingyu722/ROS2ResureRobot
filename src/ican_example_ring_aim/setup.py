from glob import glob
from setuptools import setup

package_name = "ican_example_ring_aim"

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
    description="Example ring target aiming node for the iCAN practice simulation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ring_aim_node = ican_example_ring_aim.ring_aim_node:main",
        ],
    },
)
