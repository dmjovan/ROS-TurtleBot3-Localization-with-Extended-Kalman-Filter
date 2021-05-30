#!/usr/bin/env python3

# Importovanje potrebnih biblioteka
import rospy
import numpy as np
import time
import pandas as pd
import sys
import tf
from tf.transformations import euler_from_quaternionpython
import math
from math import pi, cos, sin
from scipy.linalg import block_diag 

# Importovanje potrebnih struktura poruka
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from laser_line_extraction.msg import LineSegment,LineSegmentList

                                                                                     
# ------------------DOHVATANJA MAPE SVETA U KOJEM SE ROBOT NALAZI--------------------
#
# Funkcija za dohvatanje mape formirane pokretanjem programa make_map.py
def loadMap():

    df = pd.read_csv('map.csv')

    N_segments = len(df)

    M = np.zeros((N_segments,2))

    M[:,0] = df['rho'].to_numpy().reshape((N_segments,1))
    M[:,1] = df['alpha'].to_numpy().reshape((N_segments,1))

    return M

# -------------FUNKCIJE ZA IMPLEMENTACIJU PROSIRENOG KALMANOVOG FILTRA---------------
#
def transitionFunction(X_prev, U, b):

    delta_sl, delta_sr = U[0,0], U[1,0]

    theta_prev = X_prev[2,0]

    koef1 = (delta_sl+delta_sr)/2
    koef2 = (delta_sr-delta_sl)/(2*b)
    
    X_est = X_prev + np.array([[koef1*cos(theta_prev+koef2)],
                               [koef1*sin(theta_prev+koef2)],
                               [koef2*2]])

    theta = X_est[2,0]

    Fx = np.array([[1, 0, -koef1*sin(theta+k2)],
                   [0, 1,  koef1*cos(theta+k2)],
                   [0, 0, 1]])

    Fu11 = 0.5*(cos(theta+k2)+1/b*sin(theta+k2)*koef1)
    Fu12 = 0.5*(cos(theta+k2)-1/b*sin(theta+k2)*koef1)
    Fu21 = 0.5*(sin(theta+k2)-1/b*cos(theta+k2)*koef1)
    Fu22 = 0.5*(sin(theta+k2)+1/b*cos(theta+k2)*koef1)
    Fu31 = -1/b
    Fu32 = 1/b

    Fu = np.array([[Fu11, Fu12],
                   [Fu21, Fu22],
                   [Fu31, Fu32]])

    return X_est, Fx, Fu

def getAprioriPrediction(X_prev, P_prev, U, b):

    X_est, Fx, Fu = transitionFunction(X_prev, U, b)

    delta_sl, delta_sr = U[0,0], U[1,0] 

    k = 0.01

    Q = np.array([[k*abs(delta_sl), 0],
                  [0, k*abs(delta_sr)]])

    P_est = np.dot(np.dot(Fx, P_prev), np.transpose(Fx)) + np.dot(np.dot(Fu, Q), np.transpose(Fu))

    return X_est, P_est

def measurementFunction(X_est, m_i):

    x_est, y_est, theta_est = X_est[0,0], X_est[0,1], X_est[0,2]

    w_rho_i, w_alpha_i = m_i[0,0],  m_i[0,1]

    z_est_i = np.array([[w_alpha_i-theta_est],
                        [w_rho_i-(x_est*cos(w_alpha_i)+y_est*sin(w_alpha_i))]]) 

    H_est = np.array([[0, 0, -1],
                      [-cos(w_alpha_i), -sin(w_alpha_i), 0]])

    return z_est_i, H_est

def associateMeasurement(X_est, P_est, Z, R, M, g):

    N_segments = M.shape[0]

    N_obs = Z.shape[1]

    H_hat = np.zeros((N_segments, 2, 3))
    V = np.zeros((N_segments, N_obs, 2))
    Sigma_IN = np.zeros((N_segments, N_obs, 2, 2))

    for i in range(N_segments):
        m_i = M[i,:]
        z_hat, H_hat_i = measurementFunction(X_est, m_i)
        H_hat[i,:,:] = H_hat_i

        for j in range(N_obs):
            z = Z[:,j].reshape(2,1)
            V[i,j,:] = z-z_hat
            Sigma_IN[i,j,:,:] = np.dot(np.dot(H_hat_i, P_est), np.traspose(H_hat_i)) + R[j,:,:]

    V_valid=[]
    H_valid=[]
    R_valid=[]

    for i in range(N_segments):
        for j in range(N_obs):

            dist = np.dot(np.dot(np.transpose(V[i,j,:]), np.linalg.inv(Sigma_IN[i,j,:,:])), V[i,j,:])

            if(dist < g**2): 
                V_valid.append(V[i,j,:])
                H_valid.append(H_hat[i,:,:])
                R_valid.append(R[j,:,:])
                

    V_valid = np.array(V_valid,dtype=float)
    H_valid = np.array(H_valid,dtype=float)
    R_valid = np.array(R_valid,dtype=float)

    # dim{V_valid} = kx(2x1)
    # dim{H_valid} = kx(2x3)
    # dim{R_valid} = kx(2x2), k<N_segments
    return V_valid, H_valid, R_valid 

def filterStep(X_est, P_est, V, H_est, R):

    P_est = P_est.astype(float)
    
    H = np.reshape(H_est,(-1,3)) # H=2*kx3
    V = np.reshape(V,(-1,1)) # V=2*kx1
    R = block_diag(*R) # R=2*kx2*k

    S = np.dot(np.dot(H, P_est), np.transpose(H)) + R

    # Kalmanovo pojacanje
    K = np.dot(np.dot(P_est, np.transpose(H)), np.linalg.inv(S))

    # Kalmanova estimacija novog stanja
    X_next= X_est + np.dot(K, V)

    P_next = np.dot((np.identity(3) - np.dot(K, H)), P_est)

    return X_next, P_next

# ----------------FUNKCIJE ZA DOHVATNJE I OBRADU MERENJA SA SENZORA-------------------
#
# Callback funkcija ugradjeni /line_segments topic
# koji u sebi ima citanje sa topic-a /scan 
def lidar_callback(data):

    global Z,R

    lines = data.line_segments  

    Z_temp = []
    R_temp = []

    for line in lines:

        Z_temp.append(np.array([line.radius, line.angle]))

        covariance = np.asarray(line.covariance)

        R_temp.append(covariance.reshape((2,2)))

    # if(len(Z_temp) == 0):
    #     sys.exit() # zaglavljen robot, odnosno ne vidi nista

    Z = np.transpose(np.array(Z_temp)) # Z.shape = 2xk
    R = np.array(R_temp) # R.shape = kx2x2

# Callback funkcija ugradjeni /odom topic 
def odom_callback(data): 

    global X_odom, P_odom, U, b

    covariance = data.pose.covariance

    P_odom_temp = np.array(covariance)

    idx = [0,1,5,6,7,11,30,31,35]

    P_odom = P_odom_temp[idx].reshape((3,3))

    pose = data.pose.pose.position
    orient = data.pose.pose.orientation

    _,_,theta = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])

    X_odom = np.transpose(np.array([pose.x,pose.y,theta]))

    # TimeStamp-ovi /joint_states topic-a imaju vremensku razliku od 33ms

    T = 0.033 #[s] 

    v = data.twist.twist.linear.x
    w = data.twist.twist.angular.z

    # b = rastojanje izmedju tockova
    v_l = (v + w*b/2)/2 # linearna brzina levog tocka 
    v_r = (v - w*b/2)/2 # linearna brzina levog tocka 

    # procenjeno sledece upravljanje, ZAVISI OD PERIODE
    U[0,0] = U[0,0] + T*v_l
    U[1,0] = U[1,0] + T*v_r
    
def joint_callback(data):

    global U

    delta_sr = data.position[0] - sr_last
    delta_sl = data.position[1] - sl_last

    sr_last = data.position[0]
    sl_last = data.position[1]

    U = np.array([delta_sl, delta_sr]).reshape(2,1)

# -------------------------KONTROLA KRETANJA ROBOTA-----------------------------------
#
# Funkcija za pomeranje robota
def move_robot(linear_vel, angular_vel):

    global vel, pub_velocities

    vel.linear.x = linear_vel
    vel.angular.z = angular_vel

    pub_velocities.publish(vel)

    return 

# Callback funkcija preko koje se vrsi pomeranje robota
def velocity_callback(data):

    try:
        inputs = data.data
        linear_vel, angular_vel = inputs.split(' ')
        linear_vel = float(linear_vel)
        angular_vel = float(angular_vel)

        move_robot(linear_vel, angular_vel)

    except:
        pass

# ------------------------------GLAVNI PROGRAM----------------------------------------
#
# Glavni program za pokretanje svih gore navedenih funkcionalnosti
if __name__ == '__main__':

    try:
        M = loadMap()

        b = 0.160 #[m]
        g = 0.1

        Z = None
        R = None

        sr_last = 0
        sl_last = 0

        U = np.zeros((2,1))
        X_odom = np.zeros((3,1))
        P_odom = np.zeros((3,3))

        rospy.init_node('lokalizacija_robota_EKF', anonymous=False)

        vel = Twist()

        pub_velocities = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        rospy.Subscriber('line_segments', LineSegmentList, lidar_callback)
        rospy.Subscriber('joint_states',JointState, joint_callback)
        rospy.Subscriber('vel_control', String, velocity_callback)
        rospy.Subscriber('odom', Odometry, odom_callback)

        X = np.zeros((3,1))
        P = P_odom

        # ceka se promena u lidar_callback funkciji
        while Z == None or R == None: 
            continue

        # prosireni Kalmanov Filtar
        while True:

            X_prev = X
            P_prev = P

            X_est, P_est = getApriori(X_prev, P_prev, U, b)
            V, H_est, R_est = associateMeasurement(X_est, P_est, Z, R, M, g)

            X_final, P_final = filterStep(X_est, P_est, V, H_est, R_est)
            
            print('--------------------------------------------------------')

            print('Pozicija :\n{} \nOdometrija:\n{}'.format(np.flatten(X_final), X_odom)) 
            print('Kovarijaciona matrica: \n{} \nOdometrija:\n{}'.format(P_final, P_odom))

            print('--------------------------------------------------------')

            U = np.zeros((2,1))

            time.sleep(0.5)

    except rospy.ROSInterruptException:
        pass
