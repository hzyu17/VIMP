## run_all_tests.py
# Author: Hongzhe Yu
# Date: 08/09/2023
# Brief: run all tests after doing changes to the code. Good habit!

import os
import os
pwd = os.getcwd()
built_test_dir = pwd + "/build/tests"

files_in_directory = os.listdir(built_test_dir)

for test in files_in_directory:
    os.system(built_test_dir + '/' + test)