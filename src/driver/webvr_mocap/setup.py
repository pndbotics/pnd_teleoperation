from setuptools import find_packages, setup

package_name = "webvr_mocap"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "shared_utils"],
    zip_safe=True,
    maintainer="lh",
    maintainer_email="clvhao@foxmail.com",
    description="WebVR motion capture package for processing VR controller data and publishing ROS transforms",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "webvr_mocap = webvr_mocap.webvr_mocap:main",
        ],
    },
)
