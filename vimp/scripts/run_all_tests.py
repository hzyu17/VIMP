## run_all_tests.py
# Author: Hongzhe Yu
# Date: 08/09/2023
# Brief: run all tests after doing changes to the code. Good habit!

import os

full_path = os.path.realpath(__file__)
path, filename = os.path.split(full_path)
vimp_root = os.path.dirname(os.path.dirname(full_path))
built_test_dir = vimp_root + "/build/tests"
files_in_directory = os.listdir(built_test_dir)

for test in files_in_directory:
    os.system(built_test_dir + '/' + test)