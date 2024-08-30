import numpy as np

# Utility Class
class Utility:
    @staticmethod
    def parse_csv(filename: str, delim = ","):
        # Load data from CSV
        """load a CSV file and return it as a list of dict items"""
        result = []
        with open(filename) as infile:
                try:
                    result = np.loadtxt(infile, delimiter=delim, dtype=str)
                except Exception as e:
                    print(e)

        # Split the data (Body Pose and Body Vel)
        bodyPose = list([np.float64(result[k,0]), np.float64(result[k,1]), np.radians(np.float64(result[k,4]))] for k in range(1,len(result)))
        bodyVel = list([np.float64(result[k,2]), np.float64(result[k,3]), np.radians(np.float64(result[k,5]))] for k in range(1,len(result)))
        
        return np.array(bodyPose), np.array(bodyVel)
    
    @staticmethod
    def Rt_2D(th):
        return np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    
    @staticmethod
    def Rt_2DCenterRef(reference, pose):
        # Transform Body -> Back to Origin (0,0) -> Rotate -> Translate Back to World Frame
        _rotated = np.zeros_like(reference)
        _rotated = reference - np.array([pose[0], pose[1]])
        for n in range(len(reference)): _rotated[n] = (Utility.Rt_2D(pose[2]) @ _rotated[n].reshape(-1,1)).reshape(1,-1)
        _rotated += np.array([pose[0], pose[1]])
        return _rotated