from setuptools import find_packages, setup

package_name = 'api_demo'

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
    maintainer='malebolge',
    maintainer_email='malebolge@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sine_pub = api_demo.sine_pub:main',
            'sine_api = api_demo.sine_api',
            'pos_pubsub = api_demo.pos_pubsub:main',
        ],
    },
)
