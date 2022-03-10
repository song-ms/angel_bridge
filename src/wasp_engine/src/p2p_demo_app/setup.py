from setuptools import setup

package_name = 'p2p_demo_app'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emr_pub = p2p_demo_app.emr_pub:main',
            'emr_sub = p2p_demo_app.emr_sub:main',
            'backpack_pub = p2p_demo_app.backpack_pub:main',
            'backpack_sub = p2p_demo_app.backpack_sub:main',
            'joystick_pub = p2p_demo_app.joystick_pub:main',
            'joystick_sub = p2p_demo_app.joystick_sub:main',  
        ],
    },
)
