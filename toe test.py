import numpy as np
import matplotlib.pyplot as plt
import math as mt


dt = 0.001 #in METRES

a_check = 14
dcheck = 0.07

a_rad = np.radians(a_check)
    
xo = dcheck*np.tan(a_rad)
yo = -dcheck
    
kval = dcheck/np.cos(a_rad)
    
xt = xo - (0.5*dt)
yt = -np.sqrt(kval**2  - xt**2)
dxy = np.sqrt( (xo - xt)**2 + ( yo - yt)**2)
    
deltaT_rad = np.arccos( 1-( dxy/(2*kval) )**2 )
deltaT_deg = np.degrees(deltaT_rad)