############################################
#         ROBOT MOVEMENT TEST CODE         #
#  Created for kinematics, dynamics, and   #
#       trajectory testing purposes        #
############################################

import yaml
import numpy as np
from os.path import dirname, join

class kinematics:
    """
    Class to do kinematics calculation on mobile robot.
    """
    def __init__(self, path):
        # Open YAML Config File
        current_dir = dirname(__file__)
        file_path = join(current_dir, path)
        with open(file_path, 'r') as file:
            self.config = yaml.safe_load(file)

        if self.config['kinematics'] == "diff":
            self.d = float(self.config["d"]) #(mm) unit
            self.r = float(self.config['wr']) #(mm) unit

        elif self.config['kinematics'] == "omni":
            # Wheel Parameter
            self.d0 = float(self.config['kinematics']["d0"]) #(mm) unit
            self.d1 = float(self.config['kinematics']["d1"]) #(mm) unit
            self.d2 = float(self.config['kinematics']["d2"]) #(mm) unit
            
            self.b0 = np.radians(float(self.config["b0"]))
            self.b1 = np.radians(float(self.config["b1"]))
            self.b2 = np.radians(float(self.config["b2"]))
            
            self.y0 = np.radians(float(self.config["y0"]))
            self.y1 = np.radians(float(self.config["y1"]))
            self.y2 = np.radians(float(self.config["y2"]))
            
            self.r = float(self.config["wr"]) #(mm) unit
            
            # Inverse Kinematics Matrix
            self.IKV = np.array([[-np.sin(self.b0+self.y0), np.cos(self.b0+self.y0), self.d0*np.cos(self.y0)],
                                 [-np.sin(self.b1+self.y1), np.cos(self.b1+self.y1), self.d1*np.cos(self.y1)],
                                 [-np.sin(self.b2+self.y2), np.cos(self.b2+self.y2), self.d2*np.cos(self.y2)]])

            # Forward Kinematics Matrix
            self.KV = np.linalg.inv(self.IKV)

    def forward(self, actual_motor):
        """Method for compute (V1,V2,V3) to (Vx, Vy, VTheta) using forward kinematics"""
        # Calculate Vx, Vy, VTheta
        wheel = actual_motor
        body = self.KV @ wheel*(1/self.pulse_per_mm)
        # body = self.KV @ wheel
        return body

    def inverse(self, cmd_rover):
        """Method for compute (Vx, Vy, VTheta) to (V1,V2,V3) using inverse kinematics"""
        # Calculate V1, V2, V3
        body = cmd_rover
        wheel = self.IKV @ body * self.pulse_per_mm
        # wheel = self.IKV @ body 
        return wheel
    
    def rotation(self, dir, input, pose_theta):
        # Local to Global Transformation Matrix
        self.T = np.array([[np.cos(pose_theta), -np.sin(pose_theta), 0],
                           [np.sin(pose_theta), np.cos(pose_theta), 0],
                           [0, 0, 1]])

        # Global to Local Transformation Matrix
        self.Tt = np.transpose(self.T)

        if (dir == "local"):
            output = self.Tt @ input
        elif (dir == "global"):
            output = self.T @ input
        
        return output
    