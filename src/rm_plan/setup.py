from setuptools import find_packages, setup

package_name = 'rm_plan'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/'+package_name,['launch/sentry_test.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mage',
    maintainer_email='2941421694@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'decision_node=rm_plan.decision_node:main',
            'fake_referee=rm_plan.fake_referee:main',
            'sentry_info_receiver=rm_plan.sentry_info_receiver:main',
        ],
    },
)
