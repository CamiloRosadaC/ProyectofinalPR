from setuptools import setup

package_name = 'galletas_classifier'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'galletas_classifier.webcam_pub',
        'galletas_classifier.classification_sub',
        'galletas_classifier.display_sub',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Clasificación de galletas con YOLOv8, OpenCV y ROS2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'webcam_pub = galletas_classifier.webcam_pub:main',
            'classification_sub = galletas_classifier.classification_sub:main',
            'display_sub = galletas_classifier.display_sub:main',
        ],
    },
)
