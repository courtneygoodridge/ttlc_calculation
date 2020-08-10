import numpy as np
from scipy.special import fresnel
import pandas as pd

def rotate(x, y, a):
    s, c = np.sin(a), np.cos(a)
    return (
            c*x - s*y,
            s*x + c*y
            )

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

def clothoid_curve(ts, v, max_yr, transition_duration):
    duration = ts[-1]
    cornering_duration = duration - 2*transition_duration
    
    s = max_yr/transition_duration

    t = ts.copy()
    e = t.searchsorted(transition_duration)
    entry = clothoid_segment(t[:e+1], s, v)

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

def save_midline(x, y, yr, bearing, dt):

    yaw_degs = np.degrees(np.unwrap(bearing))
    yawrate = np.diff(yaw_degs, prepend = 0) / dt #in degs per second
    midline = pd.DataFrame({'x': x, 'y':y, 'yaw': np.degrees(np.unwrap(bearing)), 'yawrate':yawrate})
    midline.to_csv(f"{np.degrees(yr):.1f}_midline.csv")

def test():
    import matplotlib.pyplot as plt
    
    speed = 8
    transition = 4
    cornering = 4
    total = 2*transition + cornering
    t = np.linspace(0, total, 1000)
    yawrates = np.radians([6,13,20])
    dt = total/len(t) #frame rate
    
    for yawrate in yawrates:
        x, y, bearing = clothoid_curve(t, 8, yawrate, transition)

        save_midline(x,y,yawrate,bearing,dt)

        plt.figure("coords")
        label = f"Cornering yaw rate {np.degrees(yawrate):.1f}"
        plt.plot(x, y, label=label)
        plt.xlabel("X position (meters)")
        plt.ylabel("Y position (meters)")
        plt.figure("orientations")
        plt.plot(t, np.degrees(np.unwrap(bearing)), label=label)
        plt.xlabel("Time (seconds)")
        plt.ylabel("Bearing (degrees)")
    plt.figure("coords")
    plt.legend()
    plt.axis('equal')
    plt.figure("orientations")
    plt.legend()
    plt.show()

    
        

if __name__ == '__main__':
    test()


