from setuptools import setup, find_packages

package_name = 'offboard_control'

setup(
    name=package_name,
    version='1.0.1',
    packages=find_packages(include=['offboard_control','offboard_control.*']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manuel Domínguez Montero',
    maintainer_email='mandominguez97@gmail.com',
    description='Offboard control framework',
    license='private',
    tests_require=['pytest'],
)