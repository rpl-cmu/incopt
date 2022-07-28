from setuptools import setup, find_packages

install_requires = [line.rstrip() for line in open("requirements.txt", "r")]

setup(
    name="incoptpy",
    version="0.0.1",
    description="",
    url="",
    packages=find_packages("src"),
    package_dir={"": "src"},
    install_requires=install_requires,
    python_requires=">=3.6",
)
