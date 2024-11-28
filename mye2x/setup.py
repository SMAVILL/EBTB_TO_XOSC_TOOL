from setuptools import setup, find_packages
import subprocess
import os
import sys


env_name = "E2XStreamline_env"

# Check if virtual environment is active
if sys.prefix == sys.base_prefix:
    print(f"Virtual environment is not activated. Creating a virtual environment named {env_name}...")
    subprocess.check_call([sys.executable, "-m", "venv", env_name])
    print(f"Virtual environment '{env_name}' created. Please activate it using:")
    if os.name == 'posix':
        print(f"source {env_name}/bin/activate")
    else:
        print(f"{env_name}\\Scripts\\activate.bat")
    sys.exit(1)

# Automatically install dependencies from requirements.txt
subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", "requirements.txt"])

setup(
    name='E2XOStream',
    version='1.0.0',
    packages=find_packages(),
    install_requires=[],  # Dependencies are handled by the requirements.txt
    author='Chetan Jadhav',
    author_email='chetan.jadhav@mercedes-benz.com',
    description='A complete package setup for the E2XOStream project.',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/yourusername/yourproject',
    license='MB',
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'Topic :: Software Development :: Build Tools',
        'License :: OSI Approved :: MB License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
    ],
    keywords='example project setup',
    python_requires='>=3.8',
    package_data={'e2xostream': ['data/*.dat']},  # Include any package data in distribution
    include_package_data=True,
    scripts=['scripts/run_script.py']  # Include any executable scripts

)



