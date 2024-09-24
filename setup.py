from setuptools import setup, find_packages
import os

# Function to read requirements from requirements.txt and handle inline comments
def parse_requirements(filename):
    with open(filename, 'r') as f:
        return [
            line.split('#', 1)[0].strip()  # Remove comments and strip whitespaces
            for line in f
            if line.strip() and not line.startswith('#')  # Ignore empty lines and full-line comments
        ]

# Function to read a file
def read(*paths):
    """Build a file path from *paths* and return the contents."""
    with open(os.path.join(*paths), 'r') as f:
        return f.read()

setup(
    name='MPC-Tracker',
    version='0.1',
    description='Python toolbox to ease the use of MPC for tracking a goal with the presence of obstacles',
    long_description=read('README.md'),
    long_description_content_type='text/markdown',
    license='GPL-3.0',
    author='Arturo Roberti',
    author_email='arturo.roberti@outlook.com',
    url='https://github.com/ArturoRoberti/MPC-Tracker',
    packages=find_packages(where="src"),                        # Finds subpackages
    package_dir={'': 'src'},
    install_requires=parse_requirements('requirements.txt'),    # Install dependencies
    # platforms=['Windows', 'Linux', 'MacOS'],                    # Should work on all platforms
    platforms=['any'],                                          # Should work on all platforms
)