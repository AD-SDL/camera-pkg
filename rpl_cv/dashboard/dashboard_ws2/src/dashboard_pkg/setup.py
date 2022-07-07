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
    description='Web Dashboard Application for RPL lab',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dash_node = dashboard_pkg.dash_node:main'
        ],
    },
)
