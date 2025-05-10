from setuptools import setup, find_packages

setup(
  name="vimp",
  version="0.1.0",
  packages=find_packages(exclude=["scripts", "build", "3rdparty", "python", "pybind"]),
  install_requires=[
    # e.g. "numpy", "open3d", ...
  ]
)
