import sys
rootpath = 'C:\\Users\\pscmgo\\OneDrive for Business\\PhD\\Project\\Experiment_Code\\silent_failures\\Orca18_Analysis\\manuscript_analysis\\'
sys.path.append(rootpath)

import numpy as np
import matplotlib.pyplot as plt
import pdb
import pandas as pd

import simTrackMaker

class vehicle:
    
    def __init__(self, initialyaw, speed, dt, yawrate_readout, Course):
            
        #self.pos = np.array([pos[0], pos[1]])
        self.yawrate_readout = yawrate_readout 
        self.playback_length = len(yawrate_readout)
        
        self.pos = np.array(Course[0])
        self.yaw = initialyaw #heading angle, radians
        self.speed = speed 
        self.dt = dt       
        self.midline = Course[1]
        self.trackorigin = Course[2]
        
        self.yawrate = 0
        
        self.pos_history = []
        self.yaw_history = []
        self.yawrate_history = []                
        self.error_history = []   
        self.closestpt_history = []         

        self.Course = Course      
        
        self.currenterror, self.closestpt = self.calculatebias()      

                # self.save_history()     
        

    def calculatebias(self):

        #TODO: cut down on processing but only selecting a window of points based on lastmidindex.
        midlinedist = np.sqrt(
            ((self.pos[0]-self.midline[:,0])**2)
            +((self.pos[1]-self.midline[:,1])**2)
            ) #get a 4000 array of distances from the midline
        idx = np.argmin(abs(midlinedist)) #find smallest difference. This is the closest index on the midline.	

        closestpt = self.midline[idx,:] #xy of closest point
        dist = midlinedist[idx] #distance from closest point				

        #Sign bias from assessing if the closest point on midline is closer to the track origin than the driver position. Since the track is an oval, closer = understeering, farther = oversteering.
        middist_from_origin = np.sqrt(
            ((closestpt[0]-self.trackorigin[0])**2)
            +((closestpt[1]-self.trackorigin[1])**2)
            )  #distance of midline to origin
        pos_from_trackorigin = np.sqrt(
            ((self.pos[0]-self.trackorigin[0])**2)
            +((self.pos[1]-self.trackorigin[1])**2)
            ) #distance of driver pos to origin
        distdiff = middist_from_origin - pos_from_trackorigin #if driver distance is greater than closest point distance, steering position should be understeering
        steeringbias = dist * np.sign(distdiff)     

        return steeringbias, closestpt


    def move_vehicle(self, newyawrate):           
        """update the position of the vehicle over timestep dt"""                        
                                 
        self.yawrate = newyawrate

        # self.yawrate = np.deg2rad(0.5) # np.random.normal(0, 0.001)

        maxheadingval = np.deg2rad(35.0) #in rads per second
        
        self.yawrate = np.clip(self.yawrate, -maxheadingval, maxheadingval)
        # print(self.yawrate)
        # self.yawrate = 0.0

        self.yaw = self.yaw + self.yawrate * self.dt  #+ np.random.normal(0, 0.005)
        
        #zrnew = znew*cos(omegaH) + xnew*sin(omegaH);
        #xrnew = xnew*cos(omegaH) - znew*sin(omegaH)

        x_change = self.speed * self.dt * np.sin(self.yaw)
        y_change = self.speed * self.dt * np.cos(self.yaw)
        
        self.pos = self.pos + np.array([x_change, y_change]) 

        self.currenterror, self.closestpt = self.calculatebias()
        
        self.save_history()
    
    def save_history(self):

        self.pos_history.append(self.pos)        
        self.yaw_history.append(self.yaw)
        self.yawrate_history.append(self.yawrate)
        self.error_history.append(self.currenterror)
        self.closestpt_history.append(self.closestpt)

    

def runSimulation(Course, yawrate_readout, myrads, yawrateoffset= 0, onsettime = 0, rt = 99):

    """run simulation and return RMS"""

    #Sim params
    fps = 60.0
    speed = 8.0
 
    yawrateoffset_rads = np.deg2rad(yawrateoffset)
   # print ("speed; ", speed)

    dt = 1.0 / fps
    run_time = 25#15 #seconds
    time = 0

    Car = vehicle(0.0, speed, dt, yawrate_readout, Course)

    i = 0
    
    crossed = False
    time_til_crossing = None
    ttlc_at_takeover = None

    f = lambda t: np.exp(-1/t)*(t > 0)
    smooth_step = lambda t: f(t)/(f(t) + f(1 - t))

    print ("playback lenght", Car.playback_length)
    while (time < run_time) and (crossed==False):

        #print i

        time += dt              


        if (i < Car.playback_length):
            newyawrate = np.deg2rad(Car.yawrate_readout[i])
        else:
            #if exceeding playback input the mean of the last 10 frames.
            newyawrate = np.mean(np.deg2rad(Car.yawrate_readout[-10:-1]))
            #newyawrate = speed / myrads


        if time > onsettime:
            time_after_onset = time - onsettime
            transition_duration = .5
            newyawrate += smooth_step(time_after_onset/transition_duration)*yawrateoffset_rads

        
            
        
        Car.move_vehicle(newyawrate)           
        
        if crossed == False and abs(Car.currenterror) > 1.5:
            time_til_crossing = time - onsettime
            crossed = True

        i += 1

    return Car, time_til_crossing
    
def plotCar(plt, Car):
    """Plot results of simulations"""

    positions = np.array(Car.pos_history)
                        
    steeringbias = np.array(Car.error_history)

    if max(abs(steeringbias)) > 1.5:
        plt.plot(positions[:,0], positions[:,1], 'ro', markersize=.1)						
    else:
        plt.plot(positions[:,0], positions[:,1], 'go', markersize=.1)			            


def load_track():

    #create straight.
    L = 16#2sec.
    myStraight  = simTrackMaker.lineStraight(startpos = [0,0], length= 16)#, texturefile='strong_edge_soft.bmp')

    #Create Bend
    myrads = 80
    myBend = simTrackMaker.lineBend(startpos = myStraight.RoadEnd, rads = myrads, x_dir = 1, road_width=3.0) 

    #midline and edges
    Course_RoadStart = myStraight.RoadStart
    Course_midline = np.vstack((myStraight.midline, myBend.midline))
    Course_OutsideLine = np.vstack((myStraight.OutsideLine, myBend.OutsideLine))
    Course_InsideLine = np.vstack((myStraight.InsideLine, myBend.InsideLine))
    Course_CurveOrigin = myBend.CurveOrigin
    
    #Temp HACK to store in list while I improve trackmaker.
    Course = [
        Course_RoadStart,
        Course_midline, Course_CurveOrigin,
        Course_OutsideLine, Course_InsideLine
        ]
#
 
    return (Course)


def sim_ttlc(onset, sab):
 
    playbackdata = pd.read_csv('C:\\Users\\pscmgo\\OneDrive for Business\\PhD\\Project\\Experiment_Code\\silent_failures\\ttlc\\6_midline.csv') 	
    yawrate_readout = playbackdata.get("YawRate_seconds")
    Course = load_track() 
    myrads = 80
    rt_ttlc = runSimulation(Course, yawrate_readout, myrads, sab, onset, rt)
    print ("SAB: ", sab, "Onset: ", onset, "Time til Crossing: ", t)

    return(rt_ttlc, onset_ttlc)


if __name__ == '__main__':
    
    """for an onsettime, autofile array, and steering bias, should return a simulated TTLC at at the point of disengagement"""

    onset = 6
    sab = 0.5
    sim_ttlc(onset, sab)

    