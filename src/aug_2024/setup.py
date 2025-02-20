from setuptools import setup

setup(
    name='aug_2024',
    packages=['aug_2024'],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'waypoint_navigator = aug_2024.waypoint_navigator:main'
        ],
    }
)