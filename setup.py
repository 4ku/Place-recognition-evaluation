#!/usr/bin/env python

from setuptools import setup, find_packages

PACKAGE_NAME = 'place_recog_eval'
VERSION = '0.0.1'
AUTHOR = 'Ivan Efremov'
AUTHOR_EMAIL = 'ef.i.a@ya.ru'
DESCRIPTION = 'Place recognition evaluation package'
PACKAGES = find_packages()
INSTALL_REQUIRES = [
    'rospy',
    'numpy',
    'cv_bridge',
    'tqdm',
    'torch'
]

setup(
    name=PACKAGE_NAME,
    version=VERSION,
    author=AUTHOR,
    author_email=AUTHOR_EMAIL,
    description=DESCRIPTION,
    packages=PACKAGES,
    install_requires=INSTALL_REQUIRES
)
