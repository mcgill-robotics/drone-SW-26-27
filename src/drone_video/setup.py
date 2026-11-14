from glob import glob
from setuptools import setup, find_packages

setup(
    name="drone_video",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/drone_video"]),
        ("share/drone_video", ["package.xml"]),
    ]
    + [
        ("share/drone_video/" + d, glob(d + "/*"))
        for d in ("config", "launch", "deployment")
    ],
    install_requires=["setuptools"],
    maintainer="McGill Robotics",
    maintainer_email="robotics@mcgill.ca",
    description="Independent GStreamer video sender",
    license="MIT",
    entry_points={
        "console_scripts": [
            "sender=drone_video.sender:main",
            "latency=drone_video.latency:main",
        ]
    },
)
