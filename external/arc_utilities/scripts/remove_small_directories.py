#!/usr/bin/env python
import argparse
import pathlib

import colorama
from colorama import Fore

from arc_utilities.conversions import parse_file_size
from arc_utilities.path_utils import rm_tree
from arc_utilities.filesystem_utils import directory_size


def main():
    colorama.init(autoreset=True)
    parser = argparse.ArgumentParser("finds directories smaller than a given size, asks for confirmation, then deletes")
<<<<<<< HEAD
    parser.add_argument('root', type=pathlib.Path, help="root directory", nargs='+')
=======
    parser.add_argument('root', type=pathlib.Path, help="root directory")
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
    parser.add_argument('size', type=str, help="size, like 10k, 20mb, etc. ")

    args = parser.parse_args()

    size_threshold_bytes = parse_file_size(args.size)

    directories_to_remove = []
<<<<<<< HEAD
    for root in args.root:
        for d in root.iterdir():
            if d.is_dir():
                size_bytes = directory_size(d)
                if size_bytes < size_threshold_bytes:
                    directories_to_remove.append(d)
=======
    for d in args.root.iterdir():
        if d.is_dir():
            size_bytes = directory_size(d)
            if size_bytes < size_threshold_bytes:
                directories_to_remove.append(d)
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default

    print("Ok to delete these directories?")
    for d in directories_to_remove:
        print(d.as_posix())
    k = input("[Y/n]")
    if k == 'n' or k == 'N':
        print(Fore.RED + "Aborting.")
        return

    print(Fore.GREEN + "Deleting.")
    for d in directories_to_remove:
        rm_tree(d)


if __name__ == '__main__':
    main()
