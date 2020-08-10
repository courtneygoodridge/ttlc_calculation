import numpy as np
import matplotlib.pyplot as plt
import pdb
import pandas as pd

from scipy.special import fresnel

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
    
if __name__ == '__main__':
    
    myrads = 76.39 # 6 deg/s max yaw rate converted into a radius
    
    def rotate(x, y, a):
        s, c = np.sin(a), np.cos(a)
        return (
            c*x - s*y,
            s*x + c*y
            )

    def straight(startpos = [0, 0], bearing = 0, time = 2, speed = 8):
        endpos = [(startpos[0] + ((time*speed) * (np.sin(bearing)))),(startpos[1]+((time*speed)*(np.cos(bearing))))]
    
        pts = 63 * 2
    
        bearing = bearing
    
        x = np.linspace(startpos[0], endpos[0], pts)
        y = np.linspace(startpos[1], endpos[1], pts) 

    
        return np.array((x, y, bearing))

    def clothoid_segment(t, s, v, x0=0, y0=0, bearing0=0):
        x, y = np.sqrt(np.pi)*v*np.array(fresnel(np.sqrt(s)*t/np.sqrt(np.pi)))/np.sqrt(s)
        bearing = s*t**2/2 + bearing0

        x, y = rotate(x, y, bearing0)

        x += x0
        y += y0

        return np.array((x, y, bearing))

    def constant_curvature_segment(t, v, yr, x0=0, y0=0, bearing0=0):
        bearing = bearing0 + t*yr
    
        x = (v*np.cos(bearing0) - v*np.cos(bearing) + x0*yr)/yr
        y = (-v*np.sin(bearing0) + v*np.sin(bearing) + y0*yr)/yr

        return np.array((x, y, bearing))

    def clothoid_curve(ts, v, max_yr, transition_duration, startpos = [0, 0]):
        duration = ts[-1]
        cornering_duration = duration - 2*transition_duration
    
        assert(transition_duration > 0)
        assert(cornering_duration > 0)
    
        s = max_yr/transition_duration

        t = ts.copy()
        e = t.searchsorted(transition_duration)
        entry = clothoid_segment(t[:e+1], s, v, x0 = startpos[0], y0 = startpos[1])

        t0, t = t[e], t[e:]
        t -= t0
        e = t.searchsorted(cornering_duration)
        cornering = constant_curvature_segment(
                t[:e+1],
                v, max_yr,
                *entry[:,-1])

        t0, t = t[e], t[e:]
        t -= t0
        outro = clothoid_segment(t[::-1], s, v)
        outro[1] *= -1
        outro[1] -= outro[1,0]
        outro[0] -= outro[0,0]
        outro[:2] = rotate(outro[0], outro[1], -outro[-1,0])
    
        outro[2] *= -1
        outro[2] -= outro[-1,0]
        outro[:2] = rotate(outro[0], outro[1], -cornering[-1,-1])
    
        outro[2] += cornering[-1,-1]
    
        outro[:2] += cornering[:2, -1].reshape(2, -1)
    
        out = np.concatenate((entry[:,:-1], cornering[:,:-1], outro), axis=1)
    
        return out
    
    def add_edge(x, y, rw, sp = [0, 0]):
        g = np.gradient([x,y], axis = 1)
        angles = np.arctan2(g[1], g[0])
        angles = angles + np.pi/2.0 #perpendicular normal. rotate counterclockwise
        unit_normals = np.array([np.cos(angles), np.sin(angles)]) #on unit circle
        unit_normals *= rw
        xl, yl = ((x + unit_normals[0]) + sp[0]), ((y  + unit_normals[1]) + sp[1])
        return([xl, yl])

    L = 16 #2 seconds
    myStraight  = simTrackMaker.lineStraight(startpos = [0,0], length= 16)

    # Clothoid parameters
    speed = 8
    transition = 4
    cornering = 4
    total = 2*transition + cornering
    ts = np.linspace(0, total, 1000)
    yaw = np.radians(6)

    #### track components
    straight1 = straight()
    clothoid = clothoid_curve(ts, speed, yaw, transition, startpos = [0, 16])
    straight2 = straight(startpos = [(clothoid[0, -1]), clothoid[1, -1]], bearing = clothoid[2, -1])

    # selecting x and y coords for track
    x = np.hstack((straight1[0], clothoid[0], straight2[0]))
    y = np.hstack((straight1[1], clothoid[1], straight2[1]))

    ##### creating bearing/yr variable
    # clothoid_bearing = clothoid[2]
    # straight_bearing = np.linspace(0, 0, 500)
    # bearing = np.hstack((straight_bearing, clothoid_bearing))
    # t = np.linspace(0, 14, 1500)
    # yr = np.degrees(bearing) / t 

    # midline
    midline	= add_edge(x, y, rw = 0)
    track_midline = np.transpose(np.vstack((midline[0], midline[1])))

    # outside
    outside = add_edge(x,y, rw = -3)
    track_outside_line = np.transpose(np.vstack((outside[0], outside[1])))

    # inside
    inside = add_edge(x,y, rw = 3)
    track_inside_line = np.transpose(np.vstack((inside[0], inside[1])))

    # max radii of bend plus coordinate in y direction
    translate = myrads * 1
    CurveOrigin = np.add(myStraight.RoadEnd, [translate,0])

    #midline and edges
    Course_RoadStart = [0,0]
    Course_midline = track_midline
    Course_OutsideLine = track_outside_line
    Course_InsideLine = track_inside_line
    Course_CurveOrigin = CurveOrigin
    
    #Store course as list
    Course = [Course_RoadStart, Course_midline, Course_CurveOrigin, Course_OutsideLine, Course_InsideLine]
    
    #onset pool times
    OnsetTimePool = np.array([1.5, 5, 8, 11]) # 1.5, 5, 8, 11

    bend_yr = np.rad2deg(8.0 / myrads) 

    yawrateoffsets = [0]
    
    print(bend_yr)
    #columns: yr_offset, onsettime, time_til_crossing
    totalrows = len(yawrateoffsets) \
            * len(OnsetTimePool)
    
    simResults = np.empty([totalrows,3]) 
    
    row_i = 0 
    playbackdata = pd.read_csv('6_midline.csv') 	
    yawrate_readout = playbackdata.get("YawRate_seconds")
    for yr_i,yr in enumerate(yawrateoffsets):  
        for onset_i, onset in enumerate(OnsetTimePool):
            Car, t = runSimulation(Course, yawrate_readout, myrads, yr, onset)
            plotCar(plt, Car)
            simResults[row_i] =  [yr, onset, t]
            print(t)
            print ("Yr: ", yr, "Onset: ", onset, "Time til Crossing: ", t)
            row_i += 1

    #matplotlib.style.use('classic')
    plt.plot(playbackdata['x'].values, playbackdata['z'].values, 'black')
    plt.plot(Course_midline[:,0], Course_midline[:,1], color = "blue")
    plt.plot(Course_InsideLine[:,0], Course_InsideLine[:,1], color = "red")
    plt.plot(Course_OutsideLine[:,0], Course_OutsideLine[:,1], color = "red")
    plotCar(plt, Car)
    plt.title("Bend max yaw rate: " + str(round(8 / np.deg2rad(myrads), 2)) + "°/s")
    #plt.ylim(0, 50)
    #plt.xlim(-5, 40)
    plt.savefig('20_yaw_rate' + '.png', dpi = 300)

    plt.show()
    
    np.savetxt("simulated_roadcrossing.csv", simResults, delimiter=",")
    print("saved")



