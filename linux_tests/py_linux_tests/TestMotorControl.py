import json
import time
import motor_control

# Load the JSON configuration from the file
with open("DeviceConfig.json", "r") as config_file:
    config_data = json.load(config_file)

# Find the device with "class": "Motors"
motors_config = None
for device in config_data.get("Devices", []):
    if device.get("class") == "Motors":
        motors_config = device
        break

if not motors_config:
    raise ValueError("No device with class 'Motors' found in the configuration")

# Convert the configuration to a JSON string
motors_config_json = json.dumps(motors_config)

# Create and set up the MotorControl object
motor = motor_control.MotorControl(motors_config["class"], motors_config_json)
motor.setup()

# Example move command as JSON
move_command = """
{
    "cmd": "motion",
    "pos":[{"a":0,"p":100.0},{"a":1,"p":100.0}], // Target positions for the axes in units
    "rel": false,              // Move to absolute positions (false) or relative (true)
    "feedrate": 100.0,                // Feedrate percentage
    "ramped": true              // Whether the motion should be ramped
}
"""

# Send the command to the MotorControl object
ret_code = motor.send_cmd_json(move_command)

# Check the result
if ret_code == motor_control.RaftRetCode.OK:
    print("Move command executed successfully.")
else:
    print("Move command failed with code:", motor_control.get_ret_code_str(ret_code))
    
# Pump the block splitter by calling loop repeatedly
last_val_ms = 0
for i in range(10000):
    new_ms = motor.get_device_info_timestamp_ms(True, True)
    if new_ms != last_val_ms:
        print("================================================ New value:", motor.get_status_json())
        last_val_ms = new_ms
    motor.loop()
    time.sleep(0.001)
    
# # Fetch binary data as a test
# ret_code, binary_data = motor.get_data_binary(123, 256)

# # Print results
# print("Return code:", motor_control.get_ret_code_str(ret_code))
# print("Binary data:", binary_data)
