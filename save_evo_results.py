import os
import sys
import subprocess

def do_evo_stuff (input_dir):
    current_dir = input_dir

    for root, dirs, files in os.walk(current_dir):
        for file in files:
            if file.startswith('trajectory'):
                kdv_traj = os.path.join(root, file)
                print(kdv_traj)
            elif file.startswith('GT_raw_modified'):
                GT_traj = os.path.join(root, file)
                print(GT_traj)

    # Specify the nested directory structure
    ape_dir = input_dir + "/ape/ape_wo_scale_correction"
    ape_dir_scale = input_dir + "/ape/ape_with_scale_correction"

    # Create nested directories
    try:
        os.makedirs(ape_dir)
        print(f"Nested directories '{ape_dir}' created successfully.")
    except FileExistsError:
        print(f"One or more directories in '{ape_dir}' already exist.")
    except PermissionError:
        print(f"Permission denied: Unable to create '{ape_dir}'.")
    except Exception as e:
        print(f"An error occurred: {e}")

        # Create nested directories
    try:
        os.makedirs(ape_dir_scale)
        print(f"Nested directories '{ape_dir_scale}' created successfully.")
    except FileExistsError:
        print(f"One or more directories in '{ape_dir_scale}' already exist.")
    except PermissionError:
        print(f"Permission denied: Unable to create '{ape_dir_scale}'.")
    except Exception as e:
        print(f"An error occurred: {e}")

    subprocess.run(["evo_ape", "tum", "-av", GT_traj, kdv_traj, "--save_plot=" + ape_dir + "/ape_yz", "--plot_mode=yz"])
    subprocess.run(["evo_ape", "tum", "-av", GT_traj, kdv_traj, "--save_plot=" + ape_dir + "/ape_xy", "--plot_mode=xy"])
    evo_results = subprocess.run(["evo_ape", "tum", "-av", GT_traj, kdv_traj, "--save_plot=" + ape_dir + "/ape"], stdout=subprocess.PIPE, text=True)
    evo_wo_scale_output = evo_results.stdout
    evo_output_file = ape_dir + "/ape.txt"
    save_output = open(evo_output_file, "w")
    save_output.write(evo_wo_scale_output)

    subprocess.run(["evo_ape", "tum", "-avs", GT_traj, kdv_traj, "--save_plot=" + ape_dir_scale + "/ape_yz", "--plot_mode=yz"])
    subprocess.run(["evo_ape", "tum", "-avs", GT_traj, kdv_traj, "--save_plot=" + ape_dir_scale + "/ape_xy", "--plot_mode=xy"])
    evo_scale_results = subprocess.run(["evo_ape", "tum", "-avs", GT_traj, kdv_traj, "--save_plot=" + ape_dir_scale + "/ape"], stdout=subprocess.PIPE, text=True)
    evo_scale_output = evo_scale_results.stdout
    evo_output_file = ape_dir_scale + "/ape.txt"
    save_scale_output = open(evo_output_file, "w")
    save_scale_output.write(evo_scale_output)

def extract_gt(input_dir):
    current_dir = input_dir

    for root, dirs, files in os.walk(current_dir):
        for file in files:
            if file.endswith('bag'):
                bag_path = os.path.join(root, file)
                print(bag_path)
        
    gt_raw_output = subprocess.run(["rostopic", "echo", "/vrpn_client_node/GeminiMini2/pose", "-b", bag_path, "-p"], stdout=subprocess.PIPE, text=True)
    output = gt_raw_output.stdout
    gt_output_file = input_dir + "/GT_raw.txt"
    save_output = open(gt_output_file, "w")
    save_output.write(output)

if __name__ == "__main__":
    
    input_dir = sys.argv[1]
    do_evo_stuff(input_dir)
