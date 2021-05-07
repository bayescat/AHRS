import numpy as np
from math import sin, cos, tan
import matplotlib.pyplot as plt

g = 9.79641227572363 

# Quaternion normalization
def norm(q):
    
    length = q.shape[0]
    
    square_sum = 0.0    
    
    #for i in range(length):
    #    square_sum = square_sum + q[i]**2 
    
    square_sum = np.sum(q**2, axis = 0)
    
    q_norm = np.sqrt(square_sum)       
    
    return q_norm

# Euler angle을 quaternion으로 변경
# Input angle unit: degree
def euler_to_quaternoin(angles):
    #roll = np.deg2rad(angles[0])
    #pitch = np.deg2rad(angles[1])
    #yaw = np.deg2rad(angles[2])
    
    roll = angles[0]
    pitch = angles[1]
    yaw = angles[2]    
    
    # Roll
    # cos(roll/2)
    cr = np.cos(roll/2.0)
    # sin(roll/2)
    sr = np.sin(roll/2.0)
    
    # Pitch
    # cos(pitch/2)
    cp = np.cos(pitch/2.0)
    # sin(pitch/2)
    sp = np.sin(pitch/2.0)
    
    # Yaw
    # cos(yaw)
    cy = np.cos(yaw/2.0)
    # sin(yaw)
    sy = np.sin(yaw/2.0)
       
    a = cr * cp * cy + sr * sp * sy
    b = sr * cp * cy - cr * sp * sy
    c = cr * sp * cy + sr * cp * sy
    d = cr * cp * sy - sr * sp * cy    
    
    q = np.array([a,b,c,d])
    q = q / norm(q)
    
    return q

# Convert from quaternion to Euler angle in radian
def quaternion_to_euler(quaternion):
    
    #quaternion = quaternion / np.linalg.norm(quaternion)    
    
    a = quaternion[0]
    b = quaternion[1]
    c = quaternion[2]
    d = quaternion[3]
    
    # tan^-1 (x1/x2) ==> arctan2(x1, x2)
    # arctan2는 radian으로 반환
    roll = np.arctan2(2.0*(a*b+c*d),1.0-2.0*(b**2+c**2))    
    yaw = np.arctan2(2.0*(a*d+b*c),1.0-2.0*(c**2+d**2))   
    
    if(2.0*(a*c-d*b) >= 1):
        pitch = np.pi/2.0
    elif(2.0*(a*c-d*b) <= -1):
        pitch = -np.pi/2.0
    else:
        pitch = np.arcsin(2.0*(a*c-d*b))        
    
    return np.array([roll, pitch, yaw])


def accelerometer_to_quaternion(accel_xyz):
    ax = accel_xyz[0]
    ay = accel_xyz[1]
    az = accel_xyz[2]
    
    # Acceleration to Euler angle
    phi = np.arctan2(ay, az);
    theta = np.arctan2((-ax), np.sqrt(ay * ay + az * az));     
    psi = 0.0
    
    euler = np.array([phi, theta, psi])
        
    # Euler to Quaternion
    quaternion = euler_to_quaternoin(euler)
    
    return quaternion
    
    
def gyro_angular_rate_to_euler_angular_rate(attitude, 
                                            gyro_angular_rate):
    phi = attitude[0]
    theta = attitude[1]
    psi  = attitude[2]
    
    c = np.array([[1, cos(phi)*tan(theta), cos(phi)*tan(theta)],
                  [0, cos(phi), -sin(phi)],
                  [0, sin(phi)/cos(theta), cos(phi)/cos(theta)]])
    
    
    pqr = np.array([[gyro_angular_rate[0]],[gyro_angular_rate[1]],[gyro_angular_rate[2]]])        
    
    euler_angular_rate = np.matmul(c, pqr)    
        
    return euler_angular_rate      

# convert acceleration to euler angle
def acceleration_to_euler_angle(accel_axayaz):
    
    g = 9.81
    
    ax = accel_axayaz[0]
    ay = accel_axayaz[1]
    az = accel_axayaz[2]
        
    phi = np.arctan(ay/(ax**2+(az-g)**2)**(0.5))
    theta = np.arctan(ax/(az-g))
    psi = 0.0
    
    euler_angle_accel = np.array([[phi],
                                  [theta],
                                  [psi]])    
    
    return euler_angle_accel
 
def PlotAttitude(title,
                 phi,
                 theta,
                 psi
                ):

    plt.figure(figsize=(20,10))
    plt.plot(time, phi *180.0/np.pi, 'r' )
    plt.plot(time, theta *180.0/np.pi, 'g')
    plt.plot(time, psi *180.0/np.pi, 'b')
    plt.title(title, fontsize = 20)
    plt.xlabel('Time(s)', fontsize=15)
    plt.ylabel('Angle(deg)', fontsize=15)
    plt.legend(['roll($\phi$)', 'pitch($\\theta$)', 'yaw($\psi$)'], fontsize=15)
    plt.grid()
    plt.show()
    
    
def PlotAttitude_(time,
                  euler_angles_in_radian,
                  title,
                ):

    phi_radian = euler_angles_in_radian[0]    # Roll
    theta_radian = euler_angles_in_radian[1]  # Pithch
    psi_radian =euler_angles_in_radian[2]     # Yaw
    
    
    plt.figure(figsize=(20,10))
    plt.plot(time, phi_radian *180.0/np.pi, 'r' )
    plt.plot(time, theta_radian *180.0/np.pi, 'g')
    plt.plot(time, psi_radian *180.0/np.pi, 'b')
    plt.title(title, fontsize = 20)
    plt.xlabel('Time(s)', fontsize=15)
    plt.ylabel('Angle(deg)', fontsize=15)
    plt.legend(['roll($\phi$)', 'pitch($\\theta$)', 'yaw($\psi$)'], fontsize=15)
    plt.grid()
    fname = title + '.png'
    plt.savefig(fname)
    plt.show()
