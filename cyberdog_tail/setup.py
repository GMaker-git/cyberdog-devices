from setuptools import setup

package_name = 'cyberdog_tail'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shark',
    maintainer_email='gmakerbenny@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cyberdog_tail_main = cyberdog_tail.CyberTailNodeMain:main'
        ],
    },
)
