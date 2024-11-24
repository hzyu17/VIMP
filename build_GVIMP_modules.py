#!/bin/bash

import sys, os
import lsb_release
from pathlib import Path
import time

HOME_DIR = Path.home()
VIMP_DIR = os.getcwd()
VIMP_BUILD_DIR = VIMP_DIR+"/build"
vimp_DIR = VIMP_DIR+"/vimp"
GVI_DIR = vimp_DIR+"/GaussianVI"
DEP_DIR = VIMP_DIR+"/dependencies"
EIGEN_DIR = DEP_DIR+"/eigen"
EIGEN_BUILD_DIR = EIGEN_DIR+"/build"
CUDA_DIR = DEP_DIR+"/cuda"
DEVICE_QUERY_DIR = CUDA_DIR+"/cuda-samples/Samples/1_Utilities/deviceQuery"

print("NOTE: GVIMP will only build correctly if you have already installed MATLAB Runtime Library R2020b")
input("Press enter to continue... ")
time.sleep(0.5)

print("Cloning GaussianVI subdirectory... ")
time.sleep(2)
os.chdir(vimp_DIR)
os.system("git clone git@github.com:hzyu17/GaussianVI.git")

print("Downloading Eigen-3.4.0... ")
print("NOTE: This will overwrite any Eigen version currently on your machine.")
input("Press enter to continue... ")
os.chdir(VIMP_DIR)
os.system("git clone https://gitlab.com/libeigen/eigen.git --branch 3.4.0 --single-branch " + EIGEN_DIR)
os.system("mkdir -p dependencies/eigen/build")
os.chdir(EIGEN_BUILD_DIR)
os.system("cmake ..")
os.system("sudo make install")

print("Downloading OpenMP... ")
time.sleep(2)
os.chdir(VIMP_DIR)
os.system("sudo apt-get install -y --ignore-missing libomp-dev")

print("Installing CUDA... ")
print("NOTE: This step requires a compatible nvidia GPU.")
print("NOTE: Assuming an x86_64 CPU architecture.")
time.sleep(2)
os.system("mkdir dependencies")
time.sleep(0.5)
os.chdir(DEP_DIR)
os.system("mkdir cuda")
os.chdir(CUDA_DIR)
machine_desc = lsb_release.get_distro_information()
ubuntu_version_split = machine_desc["DESCRIPTION"].split(" ")[1].split(".")
ubuntu_version = ubuntu_version_split[0] + ubuntu_version_split[1]
os.system("wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu"+ubuntu_version+"/x86_64/cuda-keyring_1.1-1_all.deb")
os.system("sudo dpkg -i cuda-keyring_1.1-1_all.deb")
os.system("sudo apt-get update")
os.system("sudo apt-get install -y cuda-toolkit-12-6")
os.system("sudo apt-get install -y nvidia-driver-535")

print("Adding CUDA path to environmental variables... ")
time.sleep(2)
os.chdir(HOME_DIR)
bashrc_file = open(".bashrc", "r")
bashrc_file_contents = bashrc_file.readlines()
for line in bashrc_file_contents:
    if "cuda-12.6" in line:
        break
else:
    bashrc_file = open(".bashrc", "a")
    bashrc_file.write("export CUDACXX=/usr/local/cuda-12.6/bin/nvcc\n")
    bashrc_file.write("export PATH=/usr/local/cuda-12.6/bin/:$PATH\n")
    bashrc_file.write("export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:$LD_LIBRARY_PATH\n")
bashrc_file.close()
os.system("source .bashrc")

print("Checking system GPU capability... ")
time.sleep(2)
os.chdir(CUDA_DIR)
os.system("git clone https://github.com/NVIDIA/cuda-samples.git")
os.chdir(DEVICE_QUERY_DIR)
os.system("make")
os.system("touch output.txt")
os.system("./deviceQuery > output.txt")

file = open("output.txt", "r")

print("Updating CMake as necessary... ")
time.sleep(2)
for line in file:
    if line.find("CUDA Capability") >= 0:
        arch_sm = line.split(":")[1].strip().replace(".", "")
        arch_sm = "sm_"+arch_sm
file.close()
os.chdir(GVI_DIR)
GVI_file = open("CMakeLists.txt", "r+")
GVI_file_contents = GVI_file.readlines()
line_num = 0
for line in GVI_file_contents:
    if "set(MATLAB_ROOT_DIR" in line:
        if os.path.isdir("/usr/local/MATLAB/MATLAB_Runtime/v99"):
            replace = line.replace("/usr/local/MATLAB/R2020b", "/usr/local/MATLAB/MATLAB_Runtime/v99")
            GVI_file_contents[line_num] = replace
    if "set(CUDA_ARCH" in line:
        replace = line.replace("sm_86", arch_sm)
        GVI_file_contents[line_num] = replace
    line_num += 1
GVI_file.seek(0)
GVI_file.truncate(0)
GVI_file.writelines(GVI_file_contents)
GVI_file.close()

print("Building VIMP... ")
time.sleep(2)
os.chdir(VIMP_DIR)
os.system("mkdir build")
os.chdir(VIMP_BUILD_DIR)
os.system("cmake ..")
os.system("sudo make install")

print("Creating/Saving GH weights... ")
time.sleep(2)
os.system("./src/generate_sigmapts")
os.system("./src/save_SparseGH_weights")