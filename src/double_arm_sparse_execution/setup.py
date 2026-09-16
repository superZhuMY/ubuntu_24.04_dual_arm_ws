from setuptools import find_packages, setup


package_name = "double_arm_sparse_execution"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/sparse_execution.yaml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="gzh",
    maintainer_email="3262393329@qq.com",
    description="Sparse feedback-gated trajectory execution for the STM32 dual arm",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "sparse_trajectory_executor = "
            "double_arm_sparse_execution.sparse_trajectory_executor:main",
        ],
    },
)
