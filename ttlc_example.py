import numpy as np
import matplotlib.pyplot as plt
import pdb
import pandas as pd

from scipy.special import fresnel

import simTrackMaker
import clothoid_curve as cc

class vehicle:
    
    def __init__(self, initialyaw, speed, dt, yawrate_readout, Course, sim_params):
            
        #self.pos = np.array([pos[0], pos[1]])
        self.yawrate_readout = yawrate_readout 
        self.playback_length = len(yawrate_readout)
        
        self.pos = np.array(Course[0])
        #print(self.pos)
        self.yaw = initialyaw #heading angle, radians
        self.speed = speed 
        self.dt = dt    
        self.time = 0
        self.midline = np.array(Course[1]).T
        #self.trackorigin = Course[2]        
        #self.rads = self.trackorigin[0]
        self.afterstraight = False

        self.midline_idx = 0

        self.yawrate = 0
        
        self.pos_history = []
        self.yaw_history = []
        self.yawrate_history = []                
        self.error_history = []   
        #self.closestpt_history = []         

        self.Course = Course      
        
        #self.currenterror = self.calculatebias(self.rads) 
        self.currenterror = 0     

        #sim parameters        
        self.yawrateoffset, self.onsettime, self.smooth, self.run_time = sim_params
        self.simulated_ttlc = np.nan
        self.autofile_i = np.nan

                # self.save_history()     

    def pythag(self,pos1, pos2):

        dist = np.sqrt(
                ((pos1[0]-pos2[0])**2)
                +((pos1[1]-pos2[1])**2)
                ) 
        return dist

    def calculate_bias_clothoid(self):

        """keeps track of midline index for bias. Does not sign bias"""
        picked = False

        midindex = self.midline_idx       
        print(midindex)

        current_dist = self.pythag(self.pos, self.midline[midindex,:])

        while not picked:
            new_dist = self.pythag(self.pos, self.midline[midindex+1,:])

            print("nd: ", new_dist)
            print("current_dist: ", current_dist)
            print("midx", midindex)

            if (new_dist > current_dist) or (midindex+1 == len(self.midline[:,0])-1):
                self.midline_idx = midindex
                bias = current_dist
                picked = True
            else:
                
                current_dist = new_dist
                midindex += 1                

        return bias


    def calculatebias(self, rads):

        """function doesn't work on clothoids"""
        #for continous error do below only on bends
        if self.afterstraight:



            #bends need to take into account both x and y
            pos_from_trackorigin = np.sqrt(
                ((self.pos[0]-self.trackorigin[0])**2)
                +((self.pos[1]-self.trackorigin[1])**2)
                ) #distance of driver pos to origin
            distdiff = rads - pos_from_trackorigin #if driver distance is greater than closest point distance, steering position should be understeering        
                        
        else:
            #straights can just take into account x
            #if np.isnan(self.pos[0]):print("nan")
            distdiff = self.pos[0]                                    
            

        #print(distdiff)
    
        return distdiff


    def move_vehicle(self, newyawrate):           
        """update the position of the vehicle over timestep dt"""                        
                                 
        self.yawrate = newyawrate

        # self.yawrate = np.deg2rad(0.5) # np.random.normal(0, 0.001)

        maxheadingval = np.deg2rad(35.25) #in rads per second
        
        self.yawrate = np.clip(self.yawrate, -maxheadingval, maxheadingval)
        # print(self.yawrate) 
        # print(self.dt)
        # print(self.yawrate)
        # self.yawrate = 0.0

        myyaw = self.yaw + self.yawrate * self.dt  #+ np.random.normal(0, 0.005)
        #if np.isnan(myyaw):
        #    print("time", self.time, "pos", self.pos, "errr", self.currenterror, "newyaw", newyawrate, 'onset', self.onsettime)                
        # print(self.yaw)
        self.yaw = myyaw

        x_change = self.speed * self.dt * np.sin(self.yaw)
        
        y_change = self.speed * self.dt * np.cos(self.yaw)
        
        self.pos = self.pos + np.array([x_change, y_change]) 

        #self.currenterror = self.calculatebias(self.rads)
        self.currenterror = self.calculate_bias_clothoid()

        ##switch to different mode of calculating bias.
        #if (self.pos[1] >= self.trackorigin[1]) and (self.afterstraight == False): self.afterstraight = True
        
        self.save_history()
    
    def save_history(self):

        self.pos_history.append(self.pos)        
        self.yaw_history.append(self.yaw)
        self.yawrate_history.append(self.yawrate)
        self.error_history.append(self.currenterror)
        #self.closestpt_history.append(self.closestpt)
        #self.update_midindex()

    

def runSimulation(Course, yawrate_readout, yawrateoffset= 0, onsettime = 0, smooth = True, run_time = 60, dt = 1/60, speed = 8):

    """run simulation"""

    #Sim params
    #fps = 60.0
    #speed = 8.0
 
    yawrateoffset_rads = np.deg2rad(yawrateoffset)
   # print ("speed; ", speed)

    #dt = 1.0 / fps
    #run_time = 50 #seconds
    time = 0

    sim_params = [yawrateoffset, onsettime, smooth, run_time]

    Car = vehicle(0.0, speed, dt, yawrate_readout, Course, sim_params)

    i = 0
    
    crossed = False
    time_til_crossing = None

    f = lambda t: np.exp(-1/t)*(t > 0)
    smooth_step = lambda t: f(t)/(f(t) + f(1 - t))

    print ("playback length", Car.playback_length)
    while (time < run_time) and (crossed==False):

        time += dt              
        Car.time = time
        
        if (i < Car.playback_length):
            newyawrate = np.deg2rad(Car.yawrate_readout[i])

        if time > onsettime: # if time is after onsetime
            time_after_onset = time - onsettime
            if smooth: 
                transition_duration = .5 # half second for automation to fail
                smooth_yaw = smooth_step(time_after_onset/transition_duration) * yawrateoffset_rads                                                    
                newyawrate += smooth_yaw # is smooth yaw rate consistent yaw rate upon the onset failure?
        else:
            newyawrate = np.deg2rad(Car.yawrate_readout[i]) # otherwise yaw rate is the same as the read out

        Car.move_vehicle(newyawrate) # move vehicle according to new yaw rate           
        
        if crossed == False and abs(Car.currenterror) >= 3: # if current error is over 3 metres, sim must have crossed line
            time_til_crossing = time - onsettime # ttlc is current time minus onset time
            crossed = True # road line has been crossed
        i += 1

    Car.simulated_ttlc = time_til_crossing
    return Car, time_til_crossing

    # see TrackSimulation.py for original working of this function (I have edited certina bits that don't make sense to me)
    
    #RMS = np.sqrt(np.mean(steeringbias**2))

    #print ("RMS: ", RMS)

def plotCar(plt, Car):
    """Plot results of simulations"""

    positions = np.array(Car.pos_history)
                        
    steeringbias = np.array(Car.error_history)

    if max(abs(steeringbias)) > 1.5:
        plt.plot(positions[:,0], positions[:,1], 'ro', markersize=.2)
        #plt.xlim(-1, 30)						
    else:
        plt.plot(positions[:,0], positions[:,1], 'go', markersize=.2)			            

def add_edge(x, y, rw, sp = [0, 0]):
        g = np.gradient([x,y], axis = 1)
        angles = np.arctan2(g[1], g[0])
        angles = angles + np.pi/2.0 #perpendicular normal. rotate counterclockwise
        unit_normals = np.array([np.cos(angles), np.sin(angles)]) #on unit circle
        unit_normals *= rw
        xl, yl = ((x + unit_normals[0]) + sp[0]), ((y  + unit_normals[1]) + sp[1])
        return([xl, yl])

def straight(startpos = [0, 0], bearing = 0, time = 2, speed = 8, dt = 1/60):
    endpos = [(startpos[0] + ((time*speed) * (np.sin(bearing)))),(startpos[1]+((time*speed)*(np.cos(bearing))))]

    pts = np.round(time / dt)

    bearing = bearing

    x = np.linspace(startpos[0], endpos[0], pts)
    y = np.linspace(startpos[1], endpos[1], pts) 
    yaw = np.repeat(bearing, pts)
    
    return np.array((x, y, yaw))
    
if __name__ == '__main__':
    
    
    L = 16 #2 seconds
    myStraight  = simTrackMaker.lineStraight(startpos = [0,0], length= 16)

    # Clothoid parameters
    speed = 8
    transition = 4
    cornering = 4
    total = 2*transition + cornering
    ts = np.linspace(0, total, 1000)
    yawrates = np.radians([6, 13, 20])
    dt = total/len(ts) #frame rate


    for yawrate in yawrates:


        #create track

        x, y, bearing = cc.clothoid_curve(ts, speed, yawrate, transition)
        
        #add straights
        y += 16
        straight1 = straight(dt = dt)
        straight2 = straight(startpos = [x[-1], y[-1]], bearing = bearing[-1], dt = dt)

        # selecting x and y coords for track
        x = np.hstack((straight1[0], x, straight2[0]))
        y = np.hstack((straight1[1], y, straight2[1]))
        yaw = np.hstack((straight1[2], bearing, straight2[2]))
        yaw_degs = np.degrees(np.unwrap(yaw))
        yawrate_degs = np.diff(yaw_degs, prepend = 0) / dt #in degs per second
        
        midline = [x,y]

        # outside
        outside = add_edge(x,y, rw = -3)
        track_outside_line = np.transpose(np.vstack((outside[0], outside[1])))

        # inside
        inside = add_edge(x,y, rw = 3)
        track_inside_line = np.transpose(np.vstack((inside[0], inside[1])))
        
        #Store course as list
        Course = [[0,0], midline]
    
        #onset pool times
        OnsetTimePool = np.array([1.5, 5, 8, 11]) # 1.5, 5, 8, 11        

        yawrateoffsets = [1,2,3,4]
        
        
        #columns: yr_offset, onsettime, time_til_crossing
        totalrows = len(yawrateoffsets) \
                * len(OnsetTimePool)
        
        simResults = np.empty([totalrows,3]) 
        
        row_i = 0 
        #playbackdata = pd.read_csv(f"{np.degrees(yawrate):.1f}_midline.csv") 	
        #yawrate_readout = playbackdata.get("yawrate")
        for yro_i,yro in enumerate(yawrateoffsets):  
            for onset_i, onset in enumerate(OnsetTimePool):
                Car, t = runSimulation(Course, yawrate_degs, yro, onset, dt = dt)
                plotCar(plt, Car)
                simResults[row_i] =  [yro, onset, t]
                print(t)
                print ("Yr: ", yro, "Onset: ", onset, "Time til Crossing: ", t)
                row_i += 1

        #matplotlib.style.use('classic')
        plt.plot(x, y, 'black')        
        plt.plot(track_inside_line[:,0], track_inside_line[:,1], color = (.8,.8,.8))
        plt.plot(track_outside_line[:,0], track_outside_line[:,1], color = (.8,.8,.8))
        plotCar(plt, Car)
        #plt.title("Bend max yaw rate: " + str(round(8 / np.deg2rad(myrads), 2)) + "°/s")
        plt.title(f"Bend max yaw rate: {np.degrees(yawrate):.1f} degs/s")
        #plt.ylim(0, 50)
        #plt.xlim(-5, 40)
        plt.savefig(f'{np.degrees(yawrate):.1f}_yaw_rate.png', dpi = 300)

        plt.show()
        
        np.savetxt("simulated_roadcrossing.csv", simResults, delimiter=",")
        print("saved")



