from setuptools import find_packages, setup

package_name = 'control_analysis'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tatsukiyano',
    maintainer_email='tatsukiyano@example.com',
    description='Control system analysis tools (Step response, Frequency analysis) for ROX2026',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'signal_injector = control_analysis.signal_injector:main',
            'auto_analyzer = control_analysis.auto_analyzer:main',
            'analyze = control_analysis.analysis_cli:main',
        ],
    },
)
