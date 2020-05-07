import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="fabrik",
    version="1.1.0",
    author="Example Author",
    author_email="atieh.merikh.nejadasl@vub.be",
    description="It is an implementation of the FABRIK‌ method",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/Atiehmerikh/FABRIK_python",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)