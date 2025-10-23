from setuptools import setup

package_name = "vr_mocap"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "scipy"],
    zip_safe=True,
    maintainer="lh",
    maintainer_email="clvhao@foxmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "vr_mocap = vr_mocap.vr_mocap:main",
        ],
    },
)
