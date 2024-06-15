from setuptools import setup

package_name = 'keyboard_control'

setup(
    name=package_name,
    version='2.4.0',
    packages=[],
    py_modules=[
        'keyboard_control'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chris Lalancette',
    maintainer_email='clalancette@openrobotics.org',
    author='Graylin Trevor Jay, Austin Hendrix',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: BSD',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='A robot-agnostic teleoperation node to convert keyboard'
                'commands to Twist messages.',
    license='BSD',
    entry_points={
        'console_scripts': [
            'keyboard_control = keyboard_control:main'
        ],
    },
)