import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import numpy as np

# Load the URDF model
urdf_path = "src/robot-description/pointfoot/PF_TRON1A/urdf/robot.urdf"
model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()

# Create a random configuration
q = pin.randomConfiguration(model)

# Perform forward kinematics
pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)

# Create a Meshcat visualizer
viz = MeshcatVisualizer(model, data)
viz.initViewer()
viz.loadViewerModel()

# Display the model in the viewer
viz.display(q)

print("Press Enter to exit...")
input()