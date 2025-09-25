from setuptools import find_packages
from setuptools import setup
from os import path

package_name = 'r2s_gw'

here = path.abspath(path.dirname(__file__))

with open(path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

with open(path.join(here, 'requirements.txt'), encoding='utf-8') as rf:
    requirements = [line.strip() for line in rf if line.strip() and not line.startswith('#')]

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
        ('share/' + package_name, ['requirements.txt']),
    ],
    install_requires=requirements,
    extras_require={
        'colcon': ['colcon-common-extensions>=0.3.0,<0.4.0'],
        'test': ['pytest'],
    },
    python_requires='>=3.10, <4.0',
    zip_safe=True,
    author='Michael Carroll',
    author_email='mjcarroll@intrinsic.ai',
    maintainer='Sean Gillen',
    maintainer_email='sgillen@nvidia.com',
    description='A TUI for ROS 2',
    long_description=long_description,
    long_description_content_type="text/markdown",
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'r2s_gw = r2s_gw.main:main',
        ],
    },
)
