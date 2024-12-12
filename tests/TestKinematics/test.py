import raft_kinematics

# Create a kinematics system
kinematics_system = raft_kinematics.RaftKinematicsSystem()

# Configure a kinematics instance (example config)
config = {
    "geom": "XYZ",
    "param1": 10,
    "param2": 20
}
kinematics_instance = raft_kinematics.RaftKinematicsSystem.create_kinematics(config)

# Call kinematics methods
result = kinematics_instance.pt_to_actuator(...)
print(result)
