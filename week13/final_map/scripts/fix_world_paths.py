#!/usr/bin/env python3
import sys
import os
import re

def fix_world_paths(world_file, package_path):
    """Replace relative paths in world file with absolute paths"""
    with open(world_file, 'r') as f:
        content = f.read()
    
    # Replace relative paths with absolute paths
    content = re.sub(
        r'file://\.\./materials/scripts',
        f'file://{package_path}/materials/scripts',
        content
    )
    content = re.sub(
        r'file://\.\./materials/textures',
        f'file://{package_path}/materials/textures',
        content
    )
    
    # Write back
    with open(world_file, 'w') as f:
        f.write(content)
    
    print(f"Fixed paths in {world_file}")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: fix_world_paths.py <world_file> <package_path>")
        sys.exit(1)
    
    world_file = sys.argv[1]
    package_path = sys.argv[2]
    fix_world_paths(world_file, package_path)








