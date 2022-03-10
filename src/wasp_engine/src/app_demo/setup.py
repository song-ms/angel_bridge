from setuptools import find_packages, setup

package_name = 'app_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
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
            'sine_app = app_demo.sine_app:main',
            'pos_app = app_demo.pos_app:main',
            'keyop_app = app_demo.keyop_app:main',
        ],
    },
)
