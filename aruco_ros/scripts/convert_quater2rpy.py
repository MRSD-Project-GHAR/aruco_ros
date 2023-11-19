import yaml
import transformations as tf

def quaternion_to_rpy(quaternion):
    matrix = tf.quaternion_matrix(quaternion)
    rpy = tf.euler_from_matrix(matrix, 'rxyz')
    return rpy

def main():
    # Read YAML file
    with open('cfg/marker_poses.yaml', 'r') as file:
        data = yaml.safe_load(file)

    output_file_str = ""
    # Write roll, pitch, yaw to a text file
    with open('marker_rpy.txt', 'w') as output_file:
        for marker in data['markers']:
            marker_id = marker['id']
            quaternion = marker['pose'][3:]
            xyz = marker['pose'][:3]
            rpy = quaternion_to_rpy(quaternion)

            # Write to the text file
            output_file_str += f"{marker_id} {xyz[0]:.4f} {xyz[1]:.4f} {xyz[2]:.4f} {rpy[1]:.4f} {rpy[2]:.4f} {rpy[0]:.4f} | "
            # output_file.write(f"{marker_id} {xyz[0]:.4f} {xyz[1]:.4f} {xyz[2]:.4f} {rpy[1]:.4f} {rpy[2]:.4f} {rpy[0]:.4f} | ")
            # output_file.write(f" {marker_id}\n")
            # output_file.write(f"Roll: {rpy[1]:.4f}, Pitch: {rpy[2]:.4f}, Yaw: {rpy[0]:.4f}\n\n")
        
        # strip last 2 characters
        output_file_str = output_file_str[:-2]
        output_file.write(output_file_str)

if __name__ == "__main__":
    print("Converting quaternions to roll, pitch, yaw...")
    print(
"""
Usage:
$ python scripts/convert_quater2rpy.py
""" 
    )
    main()
    
    print("Done!")


