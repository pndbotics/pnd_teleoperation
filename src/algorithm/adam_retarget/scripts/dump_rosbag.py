#!/usr/bin/env python3
import argparse
import os


def rosbag_iterator(rosbag_folder):
    """
    Generator to iterate through all .db3 files in the given folder.
    """
    for root, dirs, files in os.walk(rosbag_folder):
        for file in files:
            if file.endswith(".db3"):
                yield os.path.join(root, file)


def process_bag_file(bag_file, output_folder):
    """
    run ros2 launch to process the bag file
    """
    # ❯ ros2 launch adam_retarget dump_bone.launch.py bag:="rosbag/rosbag2_2025_03_17-11_26_37" dump_folder:="output"
    os.system(
        f"ros2 launch adam_retarget dump_bone.launch.py bag:={bag_file} dump_folder:={output_folder}"
    )


def main():
    parser = argparse.ArgumentParser(
        description="Process ROS bag files from a folder and store results in an output folder."
    )
    parser.add_argument(
        "rosbag_folder", type=str, help="Path to the folder containing ROS bag files."
    )
    parser.add_argument(
        "output_folder",
        type=str,
        help="Path to the folder where output will be stored.",
    )

    args = parser.parse_args()

    rosbag_folder = args.rosbag_folder
    output_folder = args.output_folder

    if not os.path.isdir(rosbag_folder):
        print(
            f"Error: ROS bag folder '{rosbag_folder}' not found or is not a directory."
        )
        return

    if not os.path.isdir(output_folder):
        print(f"Output folder '{output_folder}' not found. Creating it.")
        try:
            os.makedirs(output_folder, exist_ok=True)
        except OSError as e:
            print(f"Error: Could not create output folder '{output_folder}': {e}")
            return

    print(f"ROS Bag Folder: {rosbag_folder}")
    print(f"Output Folder: {output_folder}")

    # mkdir of output folder
    try:
        os.makedirs(output_folder, exist_ok=True)
    except OSError as e:
        print(f"Error: Could not create output folder '{output_folder}': {e}")
        return
    # Process each bag file in the folder
    # walk through the directory to find all .db3 files
    for bag_file in rosbag_iterator(rosbag_folder):
        print("" + "=" * 40)
        print(f"Processing bag file: {bag_file}")
        output_i_folder = os.path.join(
            output_folder, os.path.basename(bag_file).replace(".db3", "")
        )
        os.makedirs(output_i_folder, exist_ok=True)

        # Here you would call the function to process the bag file
        # For example, you might want to call a function like:
        process_bag_file(bag_file, output_i_folder)
        # This is a placeholder for actual processing logic.
        # For now, we just print the file name.
        print(f"Output will be stored in: {output_i_folder}")


if __name__ == "__main__":
    main()
