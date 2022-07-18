from setuptools import setup

package_name = "dashboard"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="amaan",
    maintainer_email="amaanite@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cam_pub = dashboard.cam_publisher:main",
            "cam_sub = dashboard.cam_subscriber:main",
            "msg_pub = dashboard.dash_msg_publisher:main",
        ],
    },
)
