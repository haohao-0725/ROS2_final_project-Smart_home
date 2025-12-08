from setuptools import setup

package_name = 'smart_home_supervisor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Smart Home Team',
    maintainer_email='maintainer@example.com',
    description='Supervisor control node for the smart home RFID system.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'supervisor_node = smart_home_supervisor.supervisor_node:main',
        ],
    },
)
