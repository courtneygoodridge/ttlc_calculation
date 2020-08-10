import sys
rootpath = 'C:\\VENLAB data\\TrackMaker\\'
sys.path.append(rootpath)

import numpy as np
import matplotlib.pyplot as plt
import pdb
import pandas as pd

import simTrackMaker

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
        self.midline = Course[1]
        self.trackorigin = Course[2]        
        self.rads = self.trackorigin[0]
        self.afterstraight = False

        self.yawrate = 0
        
        self.pos_history = []
        self.yaw_history = []
        self.yawrate_history = []                
        self.error_history = []   
        #self.closestpt_history = []         

        self.Course = Course      
        
        self.currenterror = self.calculatebias(self.rads)      

        #sim parameters        
        self.yawrateoffset, self.onsettime, self.smooth, self.run_time = sim_params
        self.simulated_ttlc = np.nan
        self.autofile_i = np.nan

                # self.save_history()     
        

    def calculatebias(self, rads):

        """
        #Below suffers from discretisation
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
        #distdiff = dist* np.sign(middist_from_origin-dist)
        """

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

        maxheadingval = np.deg2rad(35.0) #in rads per second
        
        self.yawrate = np.clip(self.yawrate, -maxheadingval, maxheadingval)
#        print(self.yawrate) 
 #       print(self.dt)
        # print(self.yawrate)
        # self.yawrate = 0.0

        myyaw = self.yaw + self.yawrate * self.dt  #+ np.random.normal(0, 0.005)
        #if np.isnan(myyaw):
        #    print("time", self.time, "pos", self.pos, "errr", self.currenterror, "newyaw", newyawrate, 'onset', self.onsettime)                
        #print(self.yaw)
        self.yaw = myyaw

        
        
        #zrnew = znew*cos(omegaH) + xnew*sin(omegaH);
        #xrnew = xnew*cos(omegaH) - znew*sin(omegaH)

        x_change = self.speed * self.dt * np.sin(self.yaw)
        
        y_change = self.speed * self.dt * np.cos(self.yaw)
        
        self.pos = self.pos + np.array([x_change, y_change]) 

        self.currenterror = self.calculatebias(self.rads)

        #switch to different mode of calculating bias.
        if (self.pos[1] >= self.trackorigin[1]) and (self.afterstraight == False): self.afterstraight = True
        
        self.save_history()
    
    def save_history(self):

        self.pos_history.append(self.pos)        
        self.yaw_history.append(self.yaw)
        self.yawrate_history.append(self.yawrate)
        self.error_history.append(self.currenterror)
        #self.closestpt_history.append(self.closestpt)

    

def runSimulation(Course, yawrate_readout, myrads, yawrateoffset= 0, onsettime = 0, smooth = True, run_time = 60):

    """run simulation"""

    #Sim params
    fps = 60.0
    speed = 8.0
 
    yawrateoffset_rads = np.deg2rad(yawrateoffset)
   # print ("speed; ", speed)

    dt = 1.0 / fps
    #run_time = 50 #seconds
    time = 0

    sim_params = [yawrateoffset, onsettime, smooth, run_time]

    Car = vehicle(0.0, speed, dt, yawrate_readout, Course, sim_params)

    i = 0
    
    crossed = False
    time_til_crossing = None

    f = lambda t: np.exp(-1/t)*(t > 0)
    smooth_step = lambda t: f(t)/(f(t) + f(1 - t))

    print ("playback lenght", Car.playback_length)
    while (time < run_time) and (crossed==False):

        #if np.isnan(Car.pos[0]):print(i)    

        time += dt              
        Car.time = time
        
        if (i < Car.playback_length):
            newyawrate = np.deg2rad(Car.yawrate_readout[i])
        else:
            #if exceeding playback just put the bend yawrate in.
            newyawrate = speed / myrads


        if time > onsettime:
            time_after_onset = time - onsettime
            if smooth:
                transition_duration = .5
                smooth_yaw = smooth_step(time_after_onset/transition_duration)*yawrateoffset_rads                                                    
                if np.isnan(smooth_yaw): #HACK, overflow issues
                    newyawrate += yawrateoffset_rads
                else:
                    newyawrate += smooth_yaw
        
            else:
                newyawrate += yawrateoffset_rads
        
            


        Car.move_vehicle(newyawrate)           
        
        if crossed == False and abs(Car.currenterror) >= 1.5:
            time_til_crossing = time - onsettime
            crossed = True

        i += 1

    Car.simulated_ttlc = time_til_crossing
    return Car, time_til_crossing
    
    #RMS = np.sqrt(np.mean(steeringbias**2))

    #print ("RMS: ", RMS)

     

def plotCar(plt, Car):
    """Plot results of simulations"""

    positions = np.array(Car.pos_history)
                        
    steeringbias = np.array(Car.error_history)

    if max(abs(steeringbias)) > 1.5:
        plt.plot(positions[:,0], positions[:,1], 'ro', markersize=.2)						
    else:
        plt.plot(positions[:,0], positions[:,1], 'go', markersize=.2)			            
    
if __name__ == '__main__':
    

    #create straight.
    L = 16#2sec.
    myStraight  = simTrackMaker.lineStraight(startpos = [0,0], length= 16)#, texturefile='strong_edge_soft.bmp')

    #Create Bend
    myrads = 80
    myBend = simTrackMaker.lineBend(startpos = myStraight.RoadEnd, rads = myrads, x_dir = 1, road_width=3.0) 

    #midline and edges
    Course_RoadStart = myStraight.RoadStart
    Course_midline = np.vstack((myStraight.midline, myBend.midline))
    Course_OutsideLine = np.vstack(
        (myStraight.OutsideLine, myBend.OutsideLine)
        )
    Course_InsideLine = np.vstack((myStraight.InsideLine, myBend.InsideLine))
    Course_CurveOrigin = myBend.CurveOrigin
    
    #Plot Bend
    plt.figure(1)
    plt.plot(Course_midline[:,0], Course_midline[:,1], '--k')
    
    xlimits = Course_CurveOrigin[0]*2
        
    plt.xlim([0-5, xlimits+5])
    plt.ylim([-Course_CurveOrigin[0]-5, Course_CurveOrigin[1]*2 + Course_CurveOrigin[0]+5])
    plt.plot(Course_OutsideLine[:,0], Course_OutsideLine[:,1],'-k')
    plt.plot(Course_InsideLine[:,0], Course_InsideLine[:,1],'-k')
    plt.axis('equal')    
    plt.title("Radius: " + str(myrads))

    #Temp HACK to store in list while I improve trackmaker.
    Course = [
        Course_RoadStart,
        Course_midline, Course_CurveOrigin,
        Course_OutsideLine, Course_InsideLine
        ]
#

    #list of filenames
    if myrads == 40:
        #filename_list = ["Midline_40_0.csv","Midline_40_1.csv","Midline_40_2.csv","Midline_40_3.csv","Midline_40_4.csv","Midline_40_5.csv"]

        filename_list = ["Midline_40_0.csv"]
		
		
    elif myrads == 80:
      #  filename_list = ["Midline_80_0.csv","Midline_80_1.csv","Midline_80_2.csv","Midline_80_3.csv","Midline_80_4.csv","Midline_80_5.csv"]

        filename_list = ["Midline_80_1.csv"]
    else:
        raise Exception('Unrecognised radius')

    #onset pool times
    #OnsetTimePool = np.arange(5, 9.25, step = .25) 

    #onset time pool = 6 s
    OnsetTimePool = [5, 7, 9]

    #yawrateoffsets = np.linspace(-4,2,1000)

    bend_yr = np.rad2deg(8.0 / myrads)
    #yawrateoffsets = [-bend_yr]
    
    yawrateoffsets = np.linspace(-bend_yr,bend_yr,500)
    print(-bend_yr)
    #columns: yr_offset, file_i, onsettime, time_til_crossing
    totalrows = len(yawrateoffsets) \
            * len(OnsetTimePool)\
            * len(filename_list)
    
    simResults = np.empty([totalrows,4]) 
    
    #self.FACTOR_YawRate_offsets = [-.2, -.05, .15, -9, -1.5, -.5].
#   need two leave (one urgent and one non-urgent), and two stay.

    row_i = 0    
    for file_i, file in enumerate(filename_list):
        #loop through filename and onset times.
        playbackdata = pd.read_csv("Data//"+file) 	
        yawrate_readout = playbackdata.get("YawRate_seconds")

        for yr_i,yr in enumerate(yawrateoffsets):        
            for onset_i, onset in enumerate(OnsetTimePool):

                Car, t = runSimulation(Course, yawrate_readout, myrads, yr, onset)
                #Plot Car
                plotCar(plt, Car)

                simResults[row_i] = [yr, file_i, onset, t]        

                print(t)

                print ("Yr: ", yr, "Onset: ", onset, "Time til Crossing: ", t)

                row_i += 1

    #plt.savefig(str(myrads) + '_Trajs.png', dpi=800)
    #plt.show()
    
    #np.savetxt("SimResults_OnsetTimes_"+str(myrads)+".csv", simResults, delimiter=",")

   # np.savetxt("SimResults_onset_6_traj_80_1.csv", simResults, delimiter=",")
    np.savetxt("simulated_roadcrossing.csv", simResults, delimiter=",")

    #plot yr and time til crossing functions.
    # plt.figure(2)
    # plt.plot(simResults[:,0], simResults[:,1], 'k-')
    # plt.xlabel("Yaw Rate Offset (deg/s)")
    # plt.ylabel("Time from Onset to Lane Crossing (s)")
    # plt.title("Radius: " + str(myrads))
    # plt.savefig(str(myrads) + '_Sims_OnsetTimes.png', dpi = 300)
    # #plt.show()
    

    