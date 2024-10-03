import json
import os
import ast

def create_map(slam_file="lab_output/slam.txt", targets_file="lab_output/targets.txt", map_save_folder=""):
    map_data = {}

    # Read and process slam file
    with open(slam_file, 'r') as f:
        usr_dict = ast.literal_eval(f.read())
        for i, tag in enumerate(usr_dict["taglist"]):
            if tag > 10:
                continue
            map_data[f"aruco{tag}_0"] = {'y': usr_dict['map'][1][i], 'x': usr_dict['map'][0][i]}
    
    # Read and process targets file
    with open(targets_file, 'r') as f:
        usr_dict = ast.literal_eval(f.read())
        for fruit, coords in usr_dict.items():
            map_data[fruit] = {'y': coords['y'], 'x': coords['x']}
    
    # Define the output map filename
    map_filename = os.path.join(map_save_folder, "slam_generated_map.txt")

    # Write map data to a text file
    with open(map_filename, 'w') as f:
        json.dump(map_data, f)
    
    return map_filename

if __name__ == "__main__":
    # Run the create_map function and save the output
    output_file = create_map()
    print(f"Map created and saved as: {output_file}")




 # # map_1.txt
    # map_base_name = 'map'
    # map_extension = '.txt'
    # map_number = 1
    # map_filename = os.path.join(map_save_folder, f"{map_base_name}_{map_number}{map_extension}")
    
    # while os.path.isfile(map_filename):
    #     map_number += 1
    #     map_filename = os.path.join(map_save_folder, f"{map_base_name}_{map_number}{map_extension}")