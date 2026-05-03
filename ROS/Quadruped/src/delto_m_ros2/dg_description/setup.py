from setuptools import setup
import os

package_name = 'dg_description'


def get_all_files(directory):
    """Recursively get all files in a directory."""
    files = []
    if not os.path.exists(directory):
        return files
    for dirpath, _, filenames in os.walk(directory):
        for f in filenames:
            # Optional: Skip hidden files
            if not f.startswith('.'):
                files.append(os.path.join(dirpath, f))
    return files


def recursive_data_files(source_root, target_root):
    """
    Recursively map files from source_root to target_root.
    Returns a list of tuples (dest_dir, [source_files]) compatible with data_files.
    """
    data_files = []
    if not os.path.exists(source_root):
        return data_files
        
    for dirpath, _, filenames in os.walk(source_root):
        # Filter out hidden files
        valid_files = [f for f in filenames if not f.startswith('.')]
        
        if valid_files:
            rel_path = os.path.relpath(dirpath, source_root)
            dest_dir = os.path.join(target_root, rel_path)
            source_files = [os.path.join(dirpath, f) for f in valid_files]
            data_files.append((dest_dir, source_files))
            
    return data_files


# Define standard data files
data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# Add launch, urdf, config directories
for subdir in ['launch', 'urdf', 'config']:
    files = get_all_files(subdir)
    if files:
        data_files.append((os.path.join('share', package_name, subdir), files))

# Add meshes recursively
# This line ensures ALL files in 'meshes' are included
data_files += recursive_data_files('meshes', os.path.join('share', package_name, 'meshes'))

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    data_files=data_files,
    zip_safe=True,
    maintainer='hongcheol',
    maintainer_email='khc@tesollo.com',
    description='The ' + package_name + ' package',
    license='BSD3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)