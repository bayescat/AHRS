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
    
    quaternion = quaternion / np.linalg.norm(quaternion)    
    
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

    plt.show()
    
    
# Auxiliary function
# Euler angle accumulation
def Euler_accum(angle, Gyroscope, Ts):
    
    mu_ = np.zeros((3)) 
    
    phi, theta, psi = angle    
    
    # Gyroscope measurement
    p, q, r = Gyroscope
    
    mu_[0] = (p + q*(sin(phi) * tan(theta)) + r*(cos(phi) * tan(theta)))*Ts
    mu_[1] =  (q*(cos(phi)) + r*(-sin(phi)))*Ts
    mu_[2] = (q *(sin(phi) * 1/cos(theta)) + r*(cos(phi) * 1/cos(theta)))*Ts
        
    mu_ = mu_ + angle
        
    return mu_


# Quaternion accumulation
def quat_accum(pre_q, Gyroscope, Ts):

    wx = Gyroscope[0]
    wy = Gyroscope[1]
    wz = Gyroscope[2]

    Omega = np.array([[0.0,-wx, -wy, -wz],[wx, 0.0, wz, -wy],[wy, -wz, 0.0, wx],[wz, wy, -wx, 0.0]])    

    q = pre_q
    
    # Quaternion rate
    dot_q = (1.0/2.0)*(np.matmul(Omega, q))    
    # Quaternion accumulation
    q = q + dot_q *Ts    
    # Quaternion normalization
    q = q/norm(q)  

    return q


# Accelerometer와 Magnitometer를 사용한 Attitude 추정
def attitudemeasure(mu, method, Accelerometer, Magnetometer=None):
    
    z_accel = Accelerometer    
    
    if method == 1:
        
        ax = z_accel[0]
        ay = z_accel[1]
        
        gravity = 9.8  # 9.8 [m/s^2]                    
        
        cosThe = np.cos(mu[1])
        z_phi = np.arcsin(-ay / (gravity * cosThe))
        z_theta = np.arcsin(ax / gravity)
        z_psi = 0.0

    else:
        
        z_accel = z_accel/np.linalg.norm(z_accel)    
            
        # Magnetometer measurement and normalization
        z_magnet = Magnetometer
        z_magnet = z_magnet/np.linalg.norm(z_magnet)            

        # Euler angle measurements 
        z_phi = np.arctan2(z_accel[1], z_accel[2])
        z_theta = np.arctan2(-z_accel[0], np.sqrt(z_accel[1]**2 + z_accel[2]**2))
        z_psi = np.arctan2(-z_magnet[1], z_magnet[0])      
        
    return np.array([z_phi, z_theta, z_psi])
