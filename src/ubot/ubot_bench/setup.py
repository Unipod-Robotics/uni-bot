import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ubot_bench'


def tree(src):
    """(install_dir, [files]) pairs for every directory under src."""
    out = []
    for root, _, files in os.walk(src):
        if files:
            out.append((os.path.join('share', package_name, root),
                        [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'requirements-bench.txt']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/experiments', glob('experiments/*.yaml')),
    ] + tree('models') + tree('docs'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Onyenweaku Chibueze',
    maintainer_email='praiseorji4@gmail.com',
    description='Low-cost indoor navigation benchmark for ubot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_model = ubot_bench.scan_model_node:main',
            'gt_publisher = ubot_bench.gt_publisher_node:main',
            'gt_map = ubot_bench.gt_map:main',
            'make_missions = ubot_bench.make_missions:main',
            'mapping_runner = ubot_bench.mapping_runner:main',
            'mission_runner = ubot_bench.mission_runner:main',
            'bench = ubot_bench.orchestrator:main',
            'bench_analyze = ubot_bench.analysis.report:main',
        ],
    },
)
