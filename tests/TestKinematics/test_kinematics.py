
import raft_kinematics

# Path to the JSON configuration file
json_file_path = "DeviceConfig.json"

try:
    # Read JSON configuration
    json_config = raft_kinematics.read_json_file(json_file_path)
    print("Loaded JSON configuration:", json_config)

    # Create the kinematics object
    kinematics = raft_kinematics.create_kinematics_from_json(json_config)
    print("Kinematics object created successfully:", kinematics)
except Exception as e:
    print("An error occurred:", e)
