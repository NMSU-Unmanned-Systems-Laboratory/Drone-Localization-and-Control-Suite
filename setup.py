from setuptools import find_packages, setup

setup(
    name='Drone_Loc_and_Cont_Suite',
    packages=find_packages(include=['Drone_Loc_and_Cont_Suite']),
    version='0.4.2',
    description='A Library for integrated localization and control using an Optitrack system with various models of UAV quadrotor crafts',
    author='KnickKnaack nicholasgrijalva01@gmail.com',
    install_requires=['pyparrot', 'djitellopy', 'rospy', 'numpy', 'simple_pid', 'zeroconf']
)