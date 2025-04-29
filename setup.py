from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'turtlebot3_agent'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'jpl-rosa>=1.0.7', # Specify ROSA version
        'python-dotenv',
        'rich',
        'pyinputplus',
        'langchain-openai', # Explicitly add if needed by your llm_config
        # Add other specific langchain dependencies if required
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROSA agent for TurtleBot3 control via Nav2',
    license='Apache License 2.0', # Or your preferred license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agent_node = turtlebot3_agent.turtlebot3_agent_node:main',
        ],
    },
)
