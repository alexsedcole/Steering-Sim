import numpy as np
import matplotlib.pyplot as plt
import math as mt


###############################################################################################


def freudenstein_equation(outer_turn_angle, L1, L2, L3, L4):
    
    theta2 = -(90-ackermann_deg) - outer_turn_angle
    
    # Convert angles to radians
    theta2_rad = np.radians(theta2)
    
    # Calculate the Freudenstein equation
    K1 = L1 / L2
    K2 = L1 / L4
    K3 = (L1**2 + L2**2 - L3**2 + L4**2) / (2 * L2 * L4)
    
    A = np.cos(theta2_rad) - K1 - K2 * np.cos(theta2_rad) + K3
    B = -2 * np.sin(theta2_rad)
    C = K1 - (K2 + 1) * np.cos(theta2_rad) + K3
    
    # Calculate theta4 using the Freudenstein equation
    theta4_rad = 2 * np.arctan2(-B + np.sqrt(B**2 - 4 * A * C), 2 * A)
    theta4 = np.degrees(theta4_rad)
    
    return theta4


########################################################################



wheel_base = 1.3
wheel_track = 0.5
mechanical_trail = 0.01
wheel_offset = 0.09
pivot_centre_distance = wheel_track - 2*wheel_offset

standard_ackermann = np.arctan((0.5*pivot_centre_distance)/wheel_base)  # in rad


#Here, could use standard ackermann value, OR input custom:

#ackermann_deg = np.degrees(standard_ackermann)  

ackermann_deg = 17.0


ackermann_rad = np.radians(ackermann_deg)  



distance = 0.100  #Select this.

#larger D has the effect of increasing the toe sensitivity, and reducing the difference between the inner and outer wheel angle


#increasing D causes the optimim ackermann angle to slightly increase



##########################

#Last year's parameters:

'''

wheel_base = 1.5 #Guestimate, cant remember where exact value is
wheel_track = 0.59  #accurate
mechanical_trail = 0.015  #accurate, from report
wheel_offset = 0.065 # estimate from CAD
ackermann_rad = 0.261237  #accurate, from report

ackermann_deg = np.degrees(ackermann_rad)

pivot_centre_distance = wheel_track - 2*wheel_offset

distance = 0.055  #accurate, from report

'''

############



u = distance*np.tan(ackermann_rad)

k = distance/np.cos(ackermann_rad)

t = pivot_centre_distance - 2*u

#Static coordinates of the four-bar linkage:

OuterConnection = [ u, round(-distance,4)]
InnerConnection = [ round(pivot_centre_distance -u ,4), round(-distance,4)]


####################
#Diagram Plot:




def plot_four_bar_linkage(turn_angle, theta4, ackermann_deg, L1, L2, L3, L4):
    
    theta2 = -(90 - ackermann_deg) - turn_angle 
    # Convert into angle convention required by function
    
    # Convert angles to radians
    theta2_rad = np.radians(theta2)
    theta4_rad = np.radians(theta4)
    
    # Calculate the positions of the joints
    A = np.array([0, 0])
    B = np.array([L2 * np.cos(theta2_rad), L2 * np.sin(theta2_rad)])  # Adjusted for positive anticlockwise
    C = np.array([L1, 0])
    D = np.array([L1 + L4 * np.cos(theta4_rad), L4 * np.sin(theta4_rad)])  # Adjusted for positive anticlockwise
    
    # Plot the four-bar linkage
    plt.figure()
    plt.plot([A[0], B[0]], [A[1], B[1]], 'ro-')
    plt.plot([B[0], D[0]], [B[1], D[1]], 'go-')
    plt.plot([D[0], C[0]], [D[1], C[1]], 'bo-')
    plt.plot([C[0], A[0]], [C[1], A[1]], 'ko-')
    
    # Label the bars
    plt.text((A[0] + B[0]) / 2 + 0.03, (A[1] + B[1]) / 2, 'L2 = k', fontsize=12, ha='center')
    plt.text((B[0] + D[0]) / 2, (B[1] + D[1]) / 2 + 0.005, 'L3 = t', fontsize=12, ha='center')
    plt.text((D[0] + C[0]) / 2 - 0.03, (D[1] + C[1]) / 2, 'L4 = k', fontsize=12, ha='center')
    plt.text((C[0] + A[0]) / 2, (C[1] + A[1]) / 2 + 0.005, 'L1 = w', fontsize=12, ha='center')
    
    # Display turn angle, ackermann angle, and value of d in the top corner
    d_value = L2 * np.cos(np.radians(ackermann_deg))
    plt.text(0.95, 0.95, f'Turn angle: {turn_angle:.2f}°\nAckermann angle: {ackermann_deg:.2f}°\nd = {d_value:.2f} m', fontsize=12, ha='right', va='top', transform=plt.gca().transAxes)
    
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Geometry Diagram')
    plt.grid(True)
    plt.axis('equal')
    plt.show()

L1 = pivot_centre_distance  # Length of bar 1
L2 = k  
L3 = t  
L4 = k  

freeze_o = 0    # turn angle for freezeframe diagram
# Note - only logical to use positive values here as we are defining the left pivot as the outside wheel - choosing -ve values would make this the inside wheel...


freeze_i = freudenstein_equation(freeze_o, L1, L2, L3, L4)




#####################


##########################################################################################
##########################################################################################


def true_inner(o, k, t, pcd,wheel_base):
    # Convert outer wheel angle to radians

    R_values = wheel_base/np.tan(np.radians(o))
    
    
    # Calculate the corresponding theta4 value using the Freudenstein equation
    theta4 = freudenstein_equation(o, pcd, k, t, k)
    
    # Calculate the true inner wheel angle
    true_inner_wheel_angle = -(theta4 - (270 - ackermann_deg))
    
    return true_inner_wheel_angle, R_values

def ideal_inner(o, wheel_base, wheel_track):
    outer_rad = np.radians(o)
    
    R = wheel_base/np.tan(outer_rad)
    
    inner_rad = np.arctan(wheel_base/(R - wheel_track))
    inner_deg = np.degrees(inner_rad)
    return inner_deg, R



#Plot 1: Diagram of four-bar linkage

plot_four_bar_linkage(freeze_o, freeze_i,ackermann_deg, L1, L2, L3, L4)

#####################

steering_range = 10 #steering angle plot range / degrees

outer_wheel_angles = np.linspace(0, steering_range, 100)

#######################

true_inner_wheel_angles = [true_inner(angle, k, t, pivot_centre_distance,wheel_base)[0] for angle in outer_wheel_angles]

ideal_inner_wheel_angles = [ideal_inner(angle, wheel_base, wheel_track)[0] for angle in outer_wheel_angles]

angle_diff = [  (true_inner_wheel_angles[i] - ideal_inner_wheel_angles[i]) for i in range(len(true_inner_wheel_angles))  ]

#can change the above to absolute value of difference

R_values = [ideal_inner(angle, wheel_base, wheel_track)[1] for angle in outer_wheel_angles]



#######################
#Plot 2: True/ideal inner wheel angle vs outer wheel angle

fig, ax1 = plt.subplots()

ax1.plot(outer_wheel_angles, true_inner_wheel_angles, label='True inner wheel angle')
ax1.plot(outer_wheel_angles, ideal_inner_wheel_angles, label='Ideal inner wheel angle')
ax1.plot(outer_wheel_angles, outer_wheel_angles, 'k--', label='Parallel steering')  # Dotted line y = x

ax1.set_xlabel('Outer wheel angle (degrees)')
ax1.set_ylabel('Inner wheel angle (degrees)')

ax1.grid(True, which='both', linestyle='--', linewidth=0.5)
ax1.legend(loc='upper left')

# Add text for ackermann angle and value of d
textstr = f'a = {ackermann_deg:.2f}°\nd = {distance:.2f} m'
props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
ax1.text(0.05, 0.75, textstr, transform=ax1.transAxes, fontsize=10,
    verticalalignment='top', bbox=props)

ax2 = ax1.twiny()
ax2.set_xlim(ax1.get_xlim())
ax2.set_xticks(outer_wheel_angles[::10])
ax2.set_xticklabels([f'{r:.2f}' for r in R_values[::10]])
ax2.set_xlabel('Corresponding turn radius (m)')

plt.show()

########################################
#Plot 3: Error in inner wheel angle vs outer wheel angle


fig, ax = plt.subplots()

ax.plot(outer_wheel_angles, angle_diff, label='Error in inner wheel angle')
ax.set_xlabel('Outer wheel angle (degrees)')
ax.set_ylabel('(True - Ideal) inner wheel angle (degrees)')
ax.grid(True, which='both', linestyle='--', linewidth=0.5)

# Add text for ackermann angle and value of d
textstr = f'a = {ackermann_deg:.2f}°\nd = {distance:.2f} m'
props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
ax.text(0.05, 0.95, textstr, transform=ax.transAxes, fontsize=10,
    verticalalignment='top', bbox=props)

ax2 = ax.twiny()
ax2.set_xlim(ax.get_xlim())
ax2.set_xticks(outer_wheel_angles[::10])
ax2.set_xticklabels([f'{r:.2f}' for r in R_values[::10]])
ax2.set_xlabel('Corresponding turn radius (m)')

# Plot 4: Error in inner wheel angle vs outer wheel angle for different ackermann values



'''

#Plot for varying d values:
def plot_error_vs_d(d_values, ackermann_rad, pivot_centre_distance, wheel_base, wheel_track, outer_wheel_angles):

    fig, ax = plt.subplots()

    for d in d_values:
        u = d * np.tan(ackermann_rad)
        k = d / np.cos(ackermann_rad)
        t = pivot_centre_distance - 2 * u
            
        true_inner_wheel_angles = [true_inner(angle, k, t, pivot_centre_distance, wheel_base)[0] for angle in outer_wheel_angles]
        ideal_inner_wheel_angles = [ideal_inner(angle, wheel_base, wheel_track)[0] for angle in outer_wheel_angles]
            
        angle_diff = [np.abs(true_inner_wheel_angles[i] - ideal_inner_wheel_angles[i]) for i in range(len(true_inner_wheel_angles))]
            
        ax.plot(outer_wheel_angles, angle_diff, label=f'd = {d:.2f} m')

    ax.set_xlabel('Outer wheel angle (degrees)')
    ax.set_ylabel('Error in inner wheel angle (degrees)')
    ax.grid(True, which='both', linestyle='--', linewidth=0.5)
    ax.legend(title='Distance d')

    # Add text for ackermann angle
    ackermann_deg = np.degrees(ackermann_rad)
    textstr = f'a = {ackermann_deg:.2f}°'
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    ax.text(0.8, 0.1, textstr, transform=ax.transAxes, fontsize=10,
            verticalalignment='top', bbox=props)

    ax2 = ax.twiny()
    ax2.set_xlim(ax.get_xlim())
    ax2.set_xticks(outer_wheel_angles[::10])
    ax2.set_xticklabels([f'{r:.2f}' for r in R_values[::10]])
    ax2.set_xlabel('Corresponding turn radius (m)')

    plt.show()


dmin = 0.05
dmax = 0.15
dstep = 0.02

d_values = np.arange(dmin, dmax + dstep, dstep)

plot_error_vs_d(d_values, ackermann_rad, pivot_centre_distance, wheel_base, wheel_track, outer_wheel_angles)

'''

##########################################



def toe_sensitivity(dt,t, d,a):
    
    a_rad = np.radians(a)
    
    xo = d*np.tan(a_rad)
    yo = -d
    
    
    xt = xo - (0.5*dt)
    yt = -np.sqrt(k**2  - xt**2)
    dxy = np.sqrt( (xo - xt)**2 + ( yo - yt)**2)
    
    deltaT_rad = np.arccos( 1-( dxy/(2*k) )**2 )
    deltaT_deg = np.degrees(deltaT_rad)
    
    sensitivity = deltaT_deg/(dt*1000)  # degree per mm
    
    #note, value of sensitivity naturally unchanged for varying dt
    
    return sensitivity, deltaT_deg

dt = 0.005

TS = toe_sensitivity(dt, t,distance,ackermann_deg)

print('Toe adjustment sensitivity:', round(TS[0],5), 'degrees per mm')
print('dt =',dt*1000,'mm:', round(TS[1],3), 'degrees')
print()


##################################################
#Test for a specific outer angle:

outer_test = 5

ideal_inner_test = ideal_inner(outer_test, wheel_base, wheel_track)
true_inner_test = true_inner(outer_test, k, t, pivot_centre_distance,wheel_base)

print('Ackermann angle:', round(ackermann_deg,3),'degrees')
print('d = ', round(distance,3),'m')
print('w (p)ivot centre distance) = ',round(pivot_centre_distance,3), 'm')
print('t (tierod length) = ',round(t,3), 'm')
print('k = ',round(k,3), 'm')
print()


print('Static Coordinates of tierod connections:')
print('Outer Connection:',OuterConnection)
print('Inner Connection:',InnerConnection)

print()
print('For an outer angle of', round(outer_test,3), 'degrees')
print('Geometric ideal inner wheel angle:', round(ideal_inner_test[0],3), 'degrees')
print('True inner wheel angle:', round(true_inner_test[0],3), 'degrees')









#Throughout, we assume the outer wheel is always at the ideal angle, and the inner angle is what deviates from ideal.

# Calculate ideal inner wheel angles based on Ackermann's formula
#Defining the outer wheel as the origin - i.e R is the distance between COR and outer wheel (not central axis of car)
#So location of inner wheel is R - wheel_track

#We expect that the difference between the ideal and true inner wheel angle is broadly indicative of cornering losses...



#The following function carries out a crude integral of the error in inner wheel angle over the range of outer wheel angles, for a range of ackermann angles.#
#The idea here is to identify which ackermann angle gives the smallest integrated error across a given range of turn angles, which we expect to be an indiaction of lowest cornering losses.


