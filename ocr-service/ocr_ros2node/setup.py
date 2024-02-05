'''
Copyright (c) 2024 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
'''
from setuptools import setup

package_name = 'ocr_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chalia',
    maintainer_email='chalia@qti.qualcomm.com',
    description='ocr service and publish result',
    license='Qualcomm-Technologies-Inc.-Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ocr_service = ocr_service.ocr_service:main',
            'ocr_server = ocr_service.ocr_server:main',
            'ocr_client = ocr_service.ocr_client:main',
            'ocr_testnode = ocr_service.ocr_testnode:main'
        ],
    },
)
