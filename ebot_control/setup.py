from setuptools import setup

package_name = 'ebot_control'

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
    maintainer='eyantra',
    maintainer_email='helpdesk@e-yantra.org',
    description='Package containing scripts to control ebot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ebot_dock = ebot_control.ebot_dock:main',
            'duplicate_imu = ebot_control.duplicate_imu:main',
            'ebot_dock_nav= ebot_control.ebot_dock_nav:main',
            'ebot_dock_service= ebot_control.ebot_dock_service:main',
            'server=ebot_control.server:main',
            'reset_all=ebot_control.reset_all:main',

        ],
    },
)
