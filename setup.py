from setuptools import setup
import os
from glob import glob

package_name = "my_package"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="somebody very awesome",
    maintainer_email="user@user.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "first_node = my_package.first_node:main",
            "second_node = my_package.second_node:main",
            "test_odom = my_package.test_odom:main",
            "test_laser = my_package.test_laser:main",
            "testa_tudo = my_package.testa_tudo:main"
        ],
    },
)
