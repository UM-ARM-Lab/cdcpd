from setuptools import setup

DESCRIPTION = """
CDCPD is a implementation of Constrained Deformable Coherent Point Drift from paper:
Occlusion-robust Deformable Object Tracking without Physics Simulation. Cheng Chi and
Dmitry Berenson.
"""

setupOpts = dict(
    name='cdcpd',
    description='Library for Constrained Deformable Coherent Point Drift',
    long_description=DESCRIPTION,
    license='MIT',
    url='https://github.com/chichengumich/cdcpd',
    author='Cheng Chi',
    author_email='chicheng@umich.edu',
)


setup(
    version='0.0.1',
    packages=['cdcpd'],
    include_package_data=True,
    install_requires=[
        "numpy",
        "scipy",
        "numexpr",
        "scikit-learn",
        "matplotlib",
        "glplotlib"
    ],
    **setupOpts
)
