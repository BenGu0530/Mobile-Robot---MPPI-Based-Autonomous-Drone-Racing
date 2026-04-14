import os
from glob import glob
from os.path import join
from setuptools import setup

package_name = "drone_models"

model_files = glob("models/**/*", recursive=True)
model_files = [f for f in model_files if not f.endswith("/") and os.path.isfile(f)]
world_files = glob("worlds/**/*", recursive=True)
world_files = [f for f in world_files if "." in f.split("/")[-1]]
urdf_files = glob("urdf/*")

setup(
    name=package_name,
    version="0.0.1",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        *[(join("share", package_name, os.path.dirname(f)), [f]) for f in model_files],
        (join("share", package_name, "worlds"), world_files),
        (join("share", package_name, "urdf"), urdf_files),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Drone models for simulation (URDF, SDF)",
    license="Apache-2.0",
    tests_require=["pytest"],
)
