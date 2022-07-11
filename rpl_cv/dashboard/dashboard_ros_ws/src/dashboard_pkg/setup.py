from setuptools import setup

package_name = 'dashboard_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Amaan Khan',
    maintainer_email='amaanite@gmail.com',
    description='ROS Package for the Web Dashboard App in the RPL',
    license='Apache Licence 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = dashboard_pkg.publisher:main'
            'listener = dashboard_pkg.subscriber:main',
        ],
    },
)
