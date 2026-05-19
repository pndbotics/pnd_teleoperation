from setuptools import setup

package_name = "pico_mocap"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "shared_utils"],
    zip_safe=True,
    maintainer="lh",
    maintainer_email="clvhao@foxmail.com",
    description="PICO VR Tracker-based mocap to TF bridge",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "pico_mocap = pico_mocap.pico_mocap:main",
        ],
    },
)
