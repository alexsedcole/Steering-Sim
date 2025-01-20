import numpy as np
import matplotlib.pyplot as plt
import math as mt

#Our parameters:

wheel_base = 1.350  #recently increased from 1300mm to 1350mm
wheel_track = 0.5

wheel_offset = 0.125
pivot_centre_distance = wheel_track - 2*wheel_offset

standard_ackermann = np.arctan((0.5*pivot_centre_distance)/wheel_base)  # in rad

#Here, could use standard ackermann value, OR select one to minimize error for inner wheel angle.

#ackermann_deg = np.degrees(standard_ackermann)  

ackermann_deg = 16.0

ackermann_rad = np.radians(ackermann_deg)  

distance = 0.09  #Select this.

#larger D has the effect of increasing the toe sensitivity, and reducing the difference between the inner and outer wheel angle

#increasing D causes the optimum ackermann angle to slightly increase

####################################################

#Last year's parameters:  (Comment out above values for this year)

# wheel_base = 1.573 #semi-accurate, from monocoque drawing
# wheel_track = 0.59  #accurate
# mechanical_trail = 0.015  #accurate, from report
# wheel_offset = 0.065 # estimate from CAD
# pivot_centre_distance = wheel_track - 2*wheel_offset

# ############
# #standard_ackermann = np.arctan((0.5*pivot_centre_distance)/wheel_base)  # in rad

# #ackermann_deg = np.rad2deg(standard_ackermann)
# ############

# ackermann_deg = 15  # 15 is optimal value, from report

# ackermann_rad = np.radians(ackermann_deg)

# distance = 0.055  #accurate, from report

################################################

u = distance*np.tan(ackermann_rad)

k = distance/np.cos(ackermann_rad)

t = pivot_centre_distance - 2*u

#Static coordinates of the four-bar linkage:

OuterConnection = [ u, round(-distance,4)]
InnerConnection = [ round(pivot_centre_distance -u ,4), round(-distance,4)]

##########################################################################################
##########################################################################################

#Calculation functions:

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

def true_inner(outer, k, t, pcd,wheel_base):
    # Convert outer wheel angle to radians

    R_values = wheel_base/np.tan(np.radians(outer))
    
    # Calculate the corresponding theta4 value using the Freudenstein equation
    theta4 = freudenstein_equation(outer, pcd, k, t, k)
    
    # Calculate the true inner wheel angle
    true_inner_wheel_angle = -(theta4 - (270 - ackermann_deg))
    
    return true_inner_wheel_angle, R_values

def ideal_inner(outer, wheel_base, wheel_track):
    
    R_inner = radius_calc(outer,wheel_base,wheel_track)[2] 
    
    inner_rad = np.arctan(wheel_base/(R_inner))
    inner_deg = np.degrees(inner_rad)
    
    R_central = radius_calc(outer,wheel_base,wheel_track)[1]
    
    return inner_deg, R_central

def radius_calc(outer,wheel_base,wheel_track):
    R_outer = wheel_base/np.tan(np.radians(outer))
    
    R_centre = R_outer - wheel_track/2
    
    R_inner = R_outer - wheel_track
    
    return R_outer, R_centre, R_inner

def calculate_delta_toe(dt, d, ackermann_deg):
    a_rad = np.radians(ackermann_deg)
    xo = d * np.tan(a_rad)
    yo = -d
    kval = d / np.cos(a_rad)
    
    xt = xo - (0.5 * dt)
    yt = -np.sqrt(kval**2 - xt**2)
    dxy = np.sqrt((xo - xt)**2 + (yo - yt)**2)
    deltaT_rad = np.arccos(1 - (dxy / (2 * kval))**2)
    
    deltaT_deg = np.degrees(deltaT_rad)
    return deltaT_deg

#Plotting functions:

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
    
    # Label the coordinates of the tierod pickup points with their coordinates
    plt.text(A[0] + 0.09, A[1] + 0.01, f'A ({A[0]:.3f}, {A[1]:.3f})', fontsize=12, ha='right')
    plt.text(B[0] +0.08, B[1]-0.02, f'B ({B[0]:.3f}, {B[1]:.3f})', fontsize=12, ha='right')
    plt.text(C[0], C[1]+0.01, f'C ({C[0]:.3f}, {C[1]:.3f})', fontsize=12, ha='right')
    plt.text(D[0], D[1] -0.02, f'D ({D[0]:.3f}, {D[1]:.3f})', fontsize=12, ha='right')
    
    # Display turn angle, ackermann angle, and value of d in the top corner
    d_value = L2 * np.cos(np.radians(ackermann_deg))
    plt.text(0.95, 0.95, f'Visualized turn angle: {turn_angle:.2f}°\nAckermann angle: {ackermann_deg:.2f}°\nd = {d_value:.2f} m', fontsize=12, ha='right', va='top', transform=plt.gca().transAxes)
    
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Geometry Diagram')
    plt.grid(True)
    plt.axis('equal')
    plt.show()

def plot_inner_wheel_angles(outer_wheel_angles, true_inner_wheel_angles, ideal_inner_wheel_angles, ackermann_deg, distance, R_values):
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

def plot_error_in_inner_wheel_angle(outer_wheel_angles, angle_diff, ackermann_deg, distance, R_values):
    fig, ax = plt.subplots()

    ax.plot(outer_wheel_angles, angle_diff, label='Error in inner wheel angle')
    ax.set_xlabel('Outer wheel angle (degrees)')
    ax.set_ylabel('(True - Ideal) inner wheel angle (degrees)')
    ax.grid(True, which='both', linestyle='--', linewidth=0.5)

    # Add text for ackermann angle and value of d
    textstr = f'a = {ackermann_deg:.2f}°\nd = {distance:.2f} m'
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    ax.text(0.8, 0.9, textstr, transform=ax.transAxes, fontsize=10,
            verticalalignment='top', bbox=props)

    ax2 = ax.twiny()
    ax2.set_xlim(ax.get_xlim())
    ax2.set_xticks(outer_wheel_angles[::10])
    ax2.set_xticklabels([f'{r:.2f}' for r in R_values[::10]])
    ax2.set_xlabel('Corresponding turn radius (m)')

    plt.show()

def plot_delta_toe_vs_dt(dt_values, dval, ackermann_deg):
    for dcheck in dval:
        D = [calculate_delta_toe(dt, dcheck, ackermann_deg) for dt in dt_values]
        plt.plot(dt_values*1000, D, label=f'd = {round(dcheck,3)} m')

    plt.xlabel('Change in tierod length (mm)')
    plt.ylabel('Change in toe (degrees)')
    plt.title('Toe as a function of change in tierod length, and design parameter d')
    plt.legend()
    plt.grid(True)
    
    textstr = f'a = {ackermann_deg:.2f}°'
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    plt.text(0.5, 0.95, textstr, transform=plt.gca().transAxes, fontsize=10,
             verticalalignment='top', horizontalalignment='right', bbox=props)
    
    plt.show()

####################################################
#Plot 1: Diagram of four-bar linkage

L1 = pivot_centre_distance  # Length of bar 1
L2 = k  
L3 = t  
L4 = k  

freeze_o = 0    # turn angle for freezeframe diagram
# Note - only logical to use positive values here as we are defining the left pivot as the outside wheel - choosing -ve values would make this the inside wheel...

freeze_i = freudenstein_equation(freeze_o, L1, L2, L3, L4)

#Plot 1: Diagram of four-bar linkage

plot_four_bar_linkage(freeze_o, freeze_i,ackermann_deg, L1, L2, L3, L4)

#####################

steering_range = 12 #steering angle plot range / degrees

outer_wheel_angles = np.linspace(0, steering_range, 100)

#######################

true_inner_wheel_angles = [true_inner(angle, k, t, pivot_centre_distance,wheel_base)[0] for angle in outer_wheel_angles]

ideal_inner_wheel_angles = [ideal_inner(angle, wheel_base, wheel_track)[0] for angle in outer_wheel_angles]

angle_diff = [  (true_inner_wheel_angles[i] - ideal_inner_wheel_angles[i]) for i in range(len(true_inner_wheel_angles))  ]

#can change the above to absolute value of difference

R_values = [ideal_inner(angle, wheel_base, wheel_track)[1] for angle in outer_wheel_angles]

#######################
#Plot 2: True/ideal inner wheel angle vs outer wheel angle

plot_inner_wheel_angles(outer_wheel_angles, true_inner_wheel_angles, ideal_inner_wheel_angles, ackermann_deg, distance, R_values)

########################################
#Plot 3: Error in inner wheel angle vs outer wheel angle

plot_error_in_inner_wheel_angle(outer_wheel_angles, angle_diff, ackermann_deg, distance, R_values)

##################################################
#Plot 4: Toe vs change in tierod length

dt_values=np.arange(0,0.01,0.001)

dval = [0.055,0.06,0.07,0.08,0.09,0.10,0.11,0.12,0.13,0.14,0.15]

plot_delta_toe_vs_dt(dt_values,dval,ackermann_deg)

#Last Year:
# d=0.055m. For dt=0.001m, change in wheel angle = 0.368 degrees

##################################################
##################################################

#Run calcs for a specific outer angle:

outer_test = 9.6

### Limiting case for regs = ~ 9.6 degrees (i.e for outer radius = 8m)

ideal_inner_test = ideal_inner(outer_test, wheel_base, wheel_track)
true_inner_test = true_inner(outer_test, k, t, pivot_centre_distance,wheel_base)
error = true_inner_test[0] - ideal_inner_test[0]

inner_r = radius_calc(outer_test,wheel_base,wheel_track)[2]
central_r = radius_calc(outer_test,wheel_base,wheel_track)[1]
outer_r = radius_calc(outer_test,wheel_base,wheel_track)[0]

##NOTE - Regulations dictate we must be able to achieve an OUTER turn radus of 8m without contacting aeroshell.

print('Run code/plotting functions for given design parameters a and d.')
print()
print('a (ackermann angle):', round(ackermann_deg,3),'degrees')
print('d (distance of pickup point behind front track) = ', round(distance,3),'m')
print('w (pivot centre distance) = ',round(pivot_centre_distance,3), 'm')
print('t (tierod length) = ',round(t,3), 'm')
print('k (distance between kingpin and pickup point) = ',round(k,3), 'm')
print()
print('Static Coordinates of tierod connections:')
print('Outer Connection:',OuterConnection)
print('Inner Connection:',InnerConnection)
print()
print('--------------------------------------------------')
print()
print('Running calcs for a specific outer wheel angle:')
print()
print('For an outer angle of', round(outer_test,3), 'degrees')
print('Geometric ideal inner wheel angle:', round(ideal_inner_test[0],3), 'degrees')
print('Actual inner wheel angle:', round(true_inner_test[0],3), 'degrees')
print('Error in inner wheel angle:', round(error,3), 'degrees')
print()
print('Inner wheel turn radius:', round(inner_r,3), 'm')
print('Central turn radius:', round(central_r,3), 'm')
print('Outer wheel turn radius:', round(outer_r,3), 'm')
print()
print('Regs dictate we must be able to achieve an OUTER turn radius of 8m without contacting aeroshell.')

#Throughout, we assume the outer wheel is always at the ideal angle, and the inner angle is what deviates from ideal.

# Calculate ideal inner wheel angles based on Ackermann's formula
#Defining the outer wheel as the origin - i.e R is the distance between COR and outer wheel (not central axis of car)
#So location of inner wheel is R - wheel_track

#We expect that the difference between the ideal and true inner wheel angle is broadly indicative of cornering losses...


