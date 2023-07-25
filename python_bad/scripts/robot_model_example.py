"""
Author: Loïc Niederhauser
Email: loic.niederhauser@epfl.ch
Last modification: 24.07.23
Version: 1.0
An example showing how to use robot model for kinematics and display
"""
import os
from pathlib import Path

from python_gp.robot_model import robotModel


def main() -> None:
    """Gets the model of the IIWA, print it and performs FK to a random link.
    """

    # Find an example URDF (iiwa7)
    urdf_path = (urdf_folder := Path(os.path.dirname(__file__)) / "../models/iiwa/") / "iiwa_description/urdf/iiwa7.urdf.xacro"

    # Load urdf
    robot_model = robotModel(urdf_folder, urdf_path)
    print(f"Loaded robot model:\n {robot_model}")

    # Perform FK
    fk_result = robot_model.fkine(q_fk := robot_model.random_q())
    fk_jacob = robot_model.jacobe(q_fk)
    
    # Show results
    print("SE3 transform matrix:\n", fk_result)
    print("Geometrical jacobian: \n", fk_jacob)
    
    # Show robot
    robot_model.plot_robot(q_fk)
    
    
main()
