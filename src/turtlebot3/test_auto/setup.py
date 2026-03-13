from setuptools import find_packages, setup

package_name = 'test_auto'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rats',
    maintainer_email='156852328+mauratoney@users.noreply.github.com',
    description='send commands to Pi for Turtlebot burger',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_auto = test_auto.test_auto:main',
	    'piPub = test_auto.piPub:main',
	    'comp_auto = test_auto.comp_auto:main',
        'scan_filter = test_auto.scan_filter:main',
        'map_go = mapAndGo.mapAndGo:main',
        'forward_back_auto = test_auto.forward_back_auto:main',
        'everything = test_auto.everything:main',
        'goToGoal = test_auto.goToGoal:main',
        'tf_pub = test_auto.tf_pub:main'
        ],
    },
)
