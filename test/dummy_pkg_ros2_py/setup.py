from setuptools import setup

package_name = 'dummy_pkg_ros2_py'

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
    maintainer='nobody',
    maintainer_email='nobody@example.com',
    description='package description',
    license='none',
)
