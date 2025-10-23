# !/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

casadi_vendor_path = get_package_prefix("casadi_vendor")

casadi_include_path = os.path.join(casadi_vendor_path, "include")
casadi_lib_path = os.path.join(casadi_vendor_path, "lib")

script_dir = os.path.dirname(os.path.realpath(__file__))
# walk through the directory and find all the .c files
c_files = []
for root, dirs, files in os.walk(script_dir):
    for file in files:
        if file.endswith(".c"):
            c_files.append(os.path.join(root, file))


def compile_c_files(c_file):
    compile_flags = [
        "-fPIC",
        "-shared",
        "-O2",
        f"-I { casadi_include_path}",
        f"-L { casadi_lib_path}",
        "-lm",
        "-lipopt",
        f"-Wl,-rpath,{casadi_lib_path}",
    ]

    output_file = c_file.replace(".c", ".so")
    compile_command = f"gcc {c_file}  -o {output_file} {' '.join(compile_flags)} "
    print(f"Compiling {c_file} to {output_file}")
    print("-" * 50)
    print("Compile command:", compile_command)

    os.system(compile_command)
    print("")


# display the list of files
print("C files to be compiled:")
for file in c_files:
    print(file)
    compile_c_files(file)
