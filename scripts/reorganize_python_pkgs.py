#!/usr/bin/env python3
import copy
import os
import sys
import shutil
from pathlib import Path

SITE_PACKAGES_DIR_NAME = 'site-packages'

def move_directory_contents(src_dir: str, dest_dir: str, ignore_files : list[str] = []):
    """
    Moves the contents of src_dir into dest_dir.

    :param src_dir: Source directory whose contents need to be moved.
    :param dest_dir: Destination directory where contents need to be moved.
    """
    # Check if source directory exists
    if not os.path.exists(src_dir):
        raise FileNotFoundError(f"Source directory '{src_dir}' does not exist.")

    # Check if destination directory exists, if not create it
    if not os.path.exists(dest_dir):
        os.makedirs(dest_dir)

    # Iterate over all the files and directories in the source directory
    print('src_dir {0}'.format(src_dir))
    for item in os.listdir(src_dir):
        src_path = os.path.join(src_dir, item)
        if item in ignore_files:
            continue

        # Move the item
        try:
            shutil.move(src_path, dest_dir)
            print(f"Moved '{src_path}' to '{dest_dir}'")
            continue
        except Exception as e:
            print(e)

def get_subdirectories(dir_path: str):
    return [str(f.path) for f in os.scandir(dir_path) if f.is_dir()]

def main():
    print('Called restructure script!')
    src_path = sys.argv[1]
    dest_path = sys.argv[2]
    print('Looking at directory {0}'.format(src_path))

    # create destination directory if it does not exists
    if not os.path.exists(dest_path):
        os.makedirs(dest_path, exist_ok=True)

    # now move everything to dest directory
    move_directory_contents(src_path, dest_path)
    shutil.rmtree(src_path)

    # now find all pip directories that need to be re-arranged
    sub_dirs_list = get_subdirectories(dest_path)
    for sub_dir in sub_dirs_list:
        sub_dir_name = Path(sub_dir).stem
        if sub_dir_name.find('pip_') == 0:
            print('Found pip directory {0} in path {1}'.format(sub_dir_name, sub_dir))

            dirs_list = get_subdirectories(sub_dir)
            site_packages_dir_path = os.path.join(sub_dir,SITE_PACKAGES_DIR_NAME)
            if site_packages_dir_path in dirs_list:
                move_directory_contents(site_packages_dir_path,
                                        dest_path,
                                        ['__init__.py'])

                # now remove mostly empty pip directory
                try:
                    shutil.rmtree(sub_dir)
                except Exception as e:
                    print('Failed to removed mostly empty directory tree {0}, {1}'.format(sub_dir, e))

if __name__ == '__main__':
    main()