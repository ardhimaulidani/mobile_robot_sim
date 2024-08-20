import os
import numpy as np
from kinematics import kinematics

class csv():
    @staticmethod
    def get_files_from_path(path: str) -> list:
        """return list of files from path"""
        result = []
        for subdir, dirs, files in os.walk(path):
            for filename in files:
                filepath = subdir + os.sep + filename
                # only return .csv files
                if filename.lower().endswith('.csv'):
                    result.append(filepath)
        return result
    
    @staticmethod
    def load_csv(filename: str, delim = ";") -> list:
        """load a CSV file and return it as a list of dict items"""
        result = []
        with open(filename) as infile:
                try:
                    result = np.loadtxt(infile, delimiter=delim, dtype=str)
                except Exception as e:
                    print(e)

        return result
            
    @staticmethod
    def load_all(path: str) -> dict:
        """loads all CSV files into a dict"""
        result = {}
        csvfiles = csv.get_files_from_path(path)
        for filename in csvfiles:
            keyname = os.path.basename(filename)[:-4]
            result[keyname] = csv.load_csv(filename, delim = ";")
        return result

class parse(csv):
    def __init__(self, filename: str):
        # Load data from CSV
        self.result = csv.load_csv(filename)
        self.GFC = None
        self.ENC = None
        self.rawENC = None
        
        # Split the data (ENC and GFC)
        ENC = []
        GFC = []
        rawENC = []
        
        for k in range(1,len(self.result)):
            # Get Timeline Correct
            time = np.float64(self.result[k,2]) - np.float64(self.result[1,2])
            if self.result[k,0] == 'real_gfc':
                # Normalize Theta
                th = np.radians(np.float32(self.result[k,10]))
                while(th>np.pi and th<-np.pi):
                    if th > np.pi : th -= 2*np.pi
                    elif th < np.pi : th += 2*np.pi
                # Append GFC array
                GFC.append([time, np.float64(self.result[k,8]), np.float64(self.result[k,9]), th])
                
                # Also Append ENC Origin
                if (k == 1):
                    rawENC.append([time, 0, 0, 0])
                    ENC.append([time, np.float64(self.result[k,8]), np.float64(self.result[k,9]), np.radians(np.float64(self.result[k,10]))])

            elif self.result[k,0] == 'real_encoder':
                ENC.append([time, np.float64(self.result[k,8]), np.float64(self.result[k,9]), np.radians(np.float64(self.result[k,10]))])
            
            elif self.result[k,0] == 'raw_encoder':
                rawENC.append([time, np.float64(self.result[k,8]), np.float64(self.result[k,9]), np.float64(self.result[k,10])])
        
        # Convert to NumPy
        self.GFC = np.array(GFC)
        self.ENC = np.array(ENC)
        self.rawENC = np.array(rawENC)
        
        # Align Timeframe
        self.t_align()
        
    def get_GFC(self):
        return self.GFC
    
    def get_ENC(self):
        return self.ENC
    
    def get_rawENC(self):
        return self.rawENC
    
    def get_ENCtoRaw(self, config_path):
        output = []
        Kinematics = kinematics(config_path)
        
        for k in range(len(self.ENC)-1):
            delta = self.ENC[k+1] - self.ENC[k]
            # Get Instanteous Velocity
            v = np.array([[delta[1]/(delta[0]*1e-3)],
                          [delta[2]/(delta[0]*1e-3)],
                          [(delta[3])/(delta[0]*1e-3)]])
            
            # Transform To Encoder Frame
            enc_frame = Kinematics.rotation("local", v, self.GFC[0,3])
            
            # Compute Angular Wheel Velocity
            result = Kinematics.inverse(Kinematics.rotation("local", enc_frame, (self.ENC[k,3]-self.GFC[0,3])))
            output.append([self.ENC[k,0], np.float32(result[0,0]), np.float32(result[1,0]), np.float32(result[2,0])])
        self.ENCtoRaw = np.array(output)
        return self.ENCtoRaw

    def get_RawtoENC(self, config_path):
        prev_th = 0
        output  = []
        output.append([np.float64(self.GFC[0,0]), np.float64(self.GFC[0,1]), np.float64(self.GFC[0,2]), np.float64(self.GFC[0,3])])
        
        result = np.array([[0.0], [0.0], [0.0]])
        
        # Get Kinematics Config
        Kinematics = kinematics(config_path)
        
        # Start Integrating
        for k in range(1, len(self.rawENC)):
            # Get Time
            dt = (self.rawENC[k,0] - self.rawENC[k-1,0]) * 1e-3
            
            # Get Instanteous Velocity
            w = np.array([[self.rawENC[k,1]],
                          [self.rawENC[k,2]],
                          [self.rawENC[k,3]]])
            
            # Convert to Encoder World Frame
            body_frame = Kinematics.rotation("global", Kinematics.rotation("global", Kinematics.forward(w), prev_th), self.GFC[0,3])
            
            # Update Position
            result += (body_frame * dt)
            
            # Normalize Theta
            if result[2,0] > np.pi : result[2,0] -= 2*np.pi
            elif result[2,0] < -np.pi : result[2,0] += 2*np.pi
            
            # Update Theta
            prev_th = result[2,0]
            
            # Append List
            output.append([self.rawENC[k,0], np.float64(result[0,0]), np.float64(result[1,0]), np.float64(result[2,0]+self.GFC[0,3])])
        self.RawtoENC = np.array(output)
        return self.RawtoENC
    
    def t_align(self):
        # ------------- Data Delta T Alignment ------------- # 
        # Iteration Update
        ticks = 0
        aligned_GFC = []
        aligned_rawENC = []
        
        # Data Trim
        while (self.GFC[-1,0] > self.rawENC[-1,0]):
            self.GFC = self.GFC[:-1]
        
        # Odometry Alignment
        for i in range(len(self.GFC)):
            # Set time to be relative with camera
            tc = self.GFC[i,0]
            ticks = tc
            
            # Search Prev Time
            while (ticks <= tc) and (ticks >= 0):
                # Define Time Sampling
                xi = self.read_rawENC_t(ticks)
                if len(xi) > 0:
                    ti = ticks
                    break
                ticks -= 1
            
            # Search Next Time
            ticks = tc
            while ticks >= tc  and (ticks <= self.rawENC[-1,0]):
                # Define Time Sampling
                xi_next = self.read_rawENC_t(ticks)
                if len(xi_next) > 0:
                    ti_next = ticks
                    break
                ticks += 1
            
            # Get Aligned Odometry Encoder
            if(ti == tc and ti_next == tc): 
                aligned_rawENC.append([tc, xi[0,0], xi[1,0], xi[2,0]])
            else: 
                xc = xi + (xi_next - xi)*((tc-ti)/(ti_next-ti))
                aligned_rawENC.append([tc, xc[0,0], xc[1,0], xc[2,0]])
            
            # Append GFC Pose
            aligned_GFC.append([tc, self.GFC[i,1], self.GFC[i,2], self.GFC[i,3]])
            
        # replace old vars
        self.GFC = np.array(aligned_GFC)
        self.rawENC = np.array(aligned_rawENC)
    
    def read_GFC_t(self, t_ms):
        ind = np.where(self.GFC[:,0] == t_ms)
        return self.GFC[ind, 1:4].reshape(-1,1)

    def read_ENCtoRaw_t(self, t_ms):
        ind = np.where(self.ENCtoRaw[:,0] == t_ms)
        return self.ENCtoRaw[ind, 1:4].reshape(-1,1) 

    def read_RawtoENC_t(self, t_ms):
        ind = np.where(self.RawtoENC[:,0] == t_ms)
        return self.RawtoENC[ind, 1:4].reshape(-1,1) 
    
    def read_rawENC_t(self, t_ms):
        ind = np.where(self.rawENC[:,0] == t_ms)
        # Choose first data if duplicate available
        if len(ind[0]) > 1:
            ind = ind[0][0]
        return self.rawENC[ind, 1:4].reshape(-1,1) 
        
    def read_ENC_t(self, t_ms):
        ind = np.where(self.ENC[:,0] == t_ms)
        return self.ENC[ind, 1:4].reshape(-1,1) 
