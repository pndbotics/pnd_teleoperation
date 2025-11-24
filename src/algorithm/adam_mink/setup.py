from glob import glob

from setuptools import find_packages, setup

package_name = "adam_mink"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=[
        "setuptools",
        "shared_utils",
        "mink>=0.0.13",
        "mujoco>=3.0.0",
        "numpy",
        "scipy",
        "pydantic>=2.12.4",
        "pyyaml>=6.0.3",
        "loop-rate-limiters>=1.2.0",
    ],
    zip_safe=True,
    maintainer="lh",
    maintainer_email="clvhao@foxmail.com",
    description="ROS2 package for Adam robot inverse kinematics using Mink solver",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "adam_mink = adam_mink.adam_mink:main",
        ],
    },
)
