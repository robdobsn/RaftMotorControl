import json
import time
import motor_control
import struct

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
cur_time_ms = 1000
motor.set_test_time_ms(cur_time_ms, 1)
motor.setup()

# Example move command as JSON
move_command = """
{
    "cmd": "motion",
    "pos":[{"a":0,"p":100.0},{"a":1,"p":100.0}], // Target positions for the axes in units
    "rel": false,              // Move to absolute positions (false) or relative (true)
    "feedrate": 100.0,                // Feedrate percentage
    "nosplit": true,
    "ramped": true              // Whether the motion should be ramped
}
"""

# Send the command to the MotorControl object
ret_code = motor.send_cmd_json(move_command)

# Check the result
if ret_code == motor_control.RaftRetCode.OK:
    print("Move request sent successfully")
else:
    print("Move request failed with code:", motor_control.get_ret_code_str(ret_code))
    
# Pump the block splitter by calling loop repeatedly
last_val_ms = 0
for i in range(20000):
    new_ms = motor.get_device_info_timestamp_ms(True, True)
    if new_ms != last_val_ms:
        motor_status_json_str = motor.get_status_json()
        try:
            motor_status = json.loads(motor_status_json_str)
        except:
            print("Error parsing JSON:", motor_status_json_str)
            break
        hex_data = motor_status.get("0", {}).get("x", "")
        data_bytes = bytes.fromhex(hex_data)
        print("Data bytes:", hex_data)
        # Unpack the data
        time_ms, steps0, pos0, steps1, pos1, steps2, pos2 = struct.unpack(">Hififif", data_bytes)
        print("Time:", time_ms, "Steps0:", steps0, "Pos0:", pos0, "Steps1:", steps1, "Pos1:", pos1)
        last_val_ms = new_ms
    motor.loop()
    cur_time_ms += 1
    motor.set_test_time_ms(cur_time_ms, 1)
    
# # Fetch binary data as a test
# ret_code, binary_data = motor.get_data_binary(123, 256)

# # Print results
# print("Return code:", motor_control.get_ret_code_str(ret_code))
# print("Binary data:", binary_data)
