from setuptools import setup, find_packages

package_name = 'jetracer_behavior'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='dev@example.com',
    description='Behavioral decision-making stack for semantic object reactions.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'semantic_behavior = jetracer_behavior.semantic_behavior:main',
            'collision_assurance = jetracer_behavior.collision_assurance:main',
            'slip_monitor = jetracer_behavior.slip_monitor:main'
        ],
    },
)
