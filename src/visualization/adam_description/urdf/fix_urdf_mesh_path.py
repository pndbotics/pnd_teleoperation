import os
import glob
import re
import pymeshlab as ml


def simplify_mesh(input_path, output_path, target_face_num):
    ms = ml.MeshSet()
    ms.load_new_mesh(input_path)
    try:
        ms.apply_filter(
            "meshing_decimation_quadric_edge_collapse",
            targetfacenum=target_face_num,
            preservenormal=True,
        )
    except ml.PyMeshLabException as e:
        print("Failed to apply filter. Error:", e)
        print("Printing available filters. Please check the correct filter name:")
        ml.print_filter_list()
        return  # Exit the function to avoid saving an unchanged mesh

    ms.save_current_mesh(output_path)


# Find all URDF files in the directory of the script
script_dir = os.path.dirname(os.path.abspath(__file__))
pattern = os.path.join(script_dir, "**", "*.urdf")
urdf_files = glob.glob(pattern, recursive=True)
print(f"Search urdf from re pattern {pattern}")
print(f"Found {len(urdf_files)} URDF files. ")

# Find all mesh files
mesh_file_type = ["stl", "dae", "obj"]
mesh_files = []
for ext in mesh_file_type:
    pattern = os.path.join(script_dir, "**", f"*.{ext.lower()}")
    mesh_files.extend(glob.glob(pattern, recursive=True))
    # Also search for uppercase extensions if necessary, or ensure mesh files use consistent casing
    pattern_upper = os.path.join(script_dir, "**", f"*.{ext.upper()}")
    # Avoid adding duplicates if a file matches both lower and upper (e.g. if OS is case-insensitive for glob)
    for f_upper in glob.glob(pattern_upper, recursive=True):
        if f_upper not in mesh_files:
            mesh_files.extend([f_upper])

print(f"Search mesh from re pattern {pattern}")
print(f"Found {len(mesh_files)} mesh files. ")
package_root_dir = os.path.abspath(os.path.dirname(script_dir))


def get_ros_package_path(path: str):
    """
    convert a abs path to ros package path
    """
    abs_path = os.path.abspath(path)
    package_relative_path = os.path.relpath(abs_path, package_root_dir)
    return f"package://adam_description/{package_relative_path}"


def replace_mesh_path(urdf_file):
    # e.g. filename="../meshes/lifting_Columns.STL"
    # Read the URDF file content
    with open(urdf_file, "r", encoding="utf-8") as file:
        urdf_content = file.read()
    # replace all filename with relative path
    match_pattern = r'filename="(\.[^"]+)"'
    for match in re.findall(match_pattern, urdf_content):
        # get the absolute path of the file
        abs_path = os.path.abspath(os.path.join(os.path.dirname(urdf_file), match))
        # convert to ros package path
        ros_package_path = get_ros_package_path(abs_path)

        print(f"replace {match} with {ros_package_path}")
        # replace the filename with the ros package path
        urdf_content = urdf_content.replace(
            f'filename="{match}"', f'filename="{ros_package_path}"'
        )
    # Write the modified content back to the URDF file
    with open(urdf_file, "w", encoding="utf-8") as file:
        file.write(urdf_content)
        print(f"Updated {urdf_file} with new mesh paths.")


# Print the found URDF files and replace the mesh paths
for urdf_file in urdf_files:
    print(urdf_file)
    replace_mesh_path(urdf_file)

# slim down the mesh files
# for mesh_file in mesh_files:
#     # get the absolute path of the file
#     abs_path = os.path.abspath(mesh_file)
#     print(f"Process mesh file: {abs_path}")
#     simplify_mesh(abs_path, abs_path, target_face_num=100000)
