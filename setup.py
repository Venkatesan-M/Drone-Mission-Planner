from setuptools import setup, find_packages

setup(
    name="DRONE-MISSION-PLANNER",
    version="1.0.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=[
        'dronekit>=2.9.2',
        'dronekit-sitl>=3.3.0',
        'pymavlink>=2.4.37',
        'matplotlib>=3.5.2',
        'numpy>=1.21.5',
        'PyYAML>=6.0',
    ],
)
