from setuptools import setup
from glob import glob

setup(
    name="piper_collision_guard", version="0.1.0",
    packages=["piper_collision_guard"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/piper_collision_guard"]),
        ("share/piper_collision_guard", ["package.xml", "README.md"]),
        ("share/piper_collision_guard/config", glob("config/*.yaml")),
        ("share/piper_collision_guard/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="idoyeon156-cloud",
    maintainer_email="275096685+idoyeon156-cloud@users.noreply.github.com",
    description="Pinocchio point-distance monitor and position-command gate for dual Piper",
    license="MIT",
    entry_points={"console_scripts": ["collision_guard = piper_collision_guard.node:main"]},
)
