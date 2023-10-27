import os
import matplotlib.pyplot as plt

home_dir = os.path.expanduser("~")
data_path = os.path.join(home_dir, ".autoware/sjtu_data/mpc_debug/data")
save_path = os.path.join(home_dir, ".autoware/sjtu_data/mpc_debug/result")

if not os.path.exists(save_path):
    os.mkdir(save_path)

item_paths = []
for root, dirs, files in os.walk(data_path):
    for dir in dirs:
        dir_path = os.path.join(root, dir)
        item_paths.append(dir_path)
        print(dir_path)
        
for it in item_paths:
    # print(it[-1])
    idx = it[-1] + ".png"
    save_dir = os.path.join(save_path, idx)
    # if not os.path.exists(save_dir):
    #     os.mkdir(save_dir)
    name_space = ["actual_traj.txt", "global_path.txt", "heading_error.txt", "lateral_error.txt"]
    heading_error = []
    lateral_error = []
    actual_path_x = []
    actual_path_y = []
    global_path_x = []
    global_path_y = []
    for name in name_space:
        txt_file_path = os.path.join(it, name)
        # print(txt_file_path)
        if not os.path.exists(txt_file_path):
            continue
        if txt_file_path.endswith("global_path.txt"):
            with open(txt_file_path, 'r') as f:
                lines = f.readlines()
                # print(lines[1][1:-1].split(','))
                data_x = lines[0][1:-2].split(',')
                data_y = lines[1][1:-1].split(',')
                for i in range(len(data_x)):
                    data_x[i] = eval(data_x[i])
                    data_y[i] = eval(data_y[i])
                # print(data_x)
                # print(data_y)
                global_path_x = data_x
                global_path_y = data_y
                # plt.plot(global_path_x, global_path_y)
                # plt.show()
        if txt_file_path.endswith("actual_traj.txt"):
            with open(txt_file_path, 'r') as f:
                lines = f.readlines()
                # print(lines)
                # print(lines[1][1:-1].split(','))
                data_x = lines[0][1:-2].split(',')
                data_y = lines[1][1:-1].split(',')
                for i in range(len(data_x)):
                    data_x[i] = eval(data_x[i])
                    data_y[i] = eval(data_y[i])
                # print(data_x)
                # print(data_y)
                actual_path_x = data_x
                actual_path_y = data_y
            
        if txt_file_path.endswith("heading_error.txt"):
            with open(txt_file_path, 'r') as f:
                lines = f.readlines()
                # print(lines)
                data = lines[0][1:-1].split(',')
                for i in range(len(data)):
                    data[i] = eval(data[i])
                heading_error = data
            
        if txt_file_path.endswith("lateral_error.txt"):
            with open(txt_file_path, 'r') as f:
                lines = f.readlines()
                # print(lines)
                data = lines[0][1:-1].split(',')
                for i in range(len(data)):
                    data[i] = eval(data[i])
                lateral_error = data
                
    plt.figure(figsize=(15, 5))
    plt.subplot(131)
    plt.plot(global_path_x, global_path_y, label="global")
    plt.plot(actual_path_x, actual_path_y, label="actual traj")
    plt.legend()
    plt.subplot(132)
    x = range(len(heading_error))
    plt.plot(x, heading_error, label= "heading err")
    plt.legend()
    plt.subplot(133)
    x = range(len(lateral_error))
    plt.plot(x, lateral_error, label = "later_err")
    plt.legend()
    plt.savefig(save_dir)
    plt.clf()
    