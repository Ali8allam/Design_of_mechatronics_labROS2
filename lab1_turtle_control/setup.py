from setuptools import find_packages, setup

package_name = "lab1_turtle_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lab1 Team",
    maintainer_email="team@example.com",
    description="Lab 1 turtlesim servers",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "go_to_pose_server = lab1_turtle_control.go_to_pose_server:main",
            "reset_service = lab1_turtle_control.reset_service:main",
            "square_action_server = lab1_turtle_control.square_action_server:main",
        ],
    },
)
