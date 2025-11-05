from setuptools import find_packages, setup

package_name = 'bt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xcao',
    maintainer_email='caoxuan8872@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             "single_sample_mapping = bt.single_sample_mapping:main",
             "multi_sample = bt.multi_sample:main",
             "measure_all_samples = bt.measure_all_samples:main",
        ],
    },
)
