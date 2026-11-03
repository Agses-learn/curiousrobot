from glob import glob
from setuptools import find_packages, setup

setup(
    name='stm32_base_bridge', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/stm32_base_bridge']),
        ('share/stm32_base_bridge', ['package.xml', 'README.md', 'LICENSE']),
        ('share/stm32_base_bridge/config', glob('config/*.yaml')),
        ('share/stm32_base_bridge/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial'], zip_safe=True,
    tests_require=['pytest'],
    maintainer='zhangkaiqi', maintainer_email='zhangkaiqi@example.com',
    description='STM32 protocol bridge for CuriousRobot', license='Apache-2.0',
    entry_points={'console_scripts': [
        'bridge_node = stm32_base_bridge.bridge_node:main',
        'mock_mcu = stm32_base_bridge.mock_mcu:main',
    ]},
)
