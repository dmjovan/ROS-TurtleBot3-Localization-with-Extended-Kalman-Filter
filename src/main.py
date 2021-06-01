#!/usr/bin/env python3

# Importovanje potrebnih biblioteka
import rospy
import numpy as np
import time
import pandas as pd
import sys
import tf
from tf.transformations import euler_from_quaternion
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

    df = pd.read_csv('Workspaces/getting_started/src/domaci_4/src/map.csv')

    N_segments = len(df)

    M = np.zeros((N_segments,2))

    M[:,0] = df['rho'].to_numpy().reshape((N_segments,))
    M[:,1] = df['alpha'].to_numpy().reshape((N_segments,))

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

    Fx = np.array([[1, 0, -koef1*sin(theta+koef2)],
                   [0, 1,  koef1*cos(theta+koef2)],
                   [0, 0, 1]])

    Fu11 = 0.5*(cos(theta+koef2)+1/b*sin(theta+koef2)*koef1)
    Fu12 = 0.5*(cos(theta+koef2)-1/b*sin(theta+koef2)*koef1)
    Fu21 = 0.5*(sin(theta+koef2)-1/b*cos(theta+koef2)*koef1)
    Fu22 = 0.5*(sin(theta+koef2)+1/b*cos(theta+koef2)*koef1)
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

    x_est, y_est, theta_est = X_est[0,0], X_est[1,0], X_est[2,0]

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
        m_i = M[i,:].reshape((1,2))
        z_hat, H_hat_i = measurementFunction(X_est, m_i)
        H_hat[i,:,:] = H_hat_i

        for j in range(N_obs):
            z = Z[:,j].reshape((2,1))
            V[i,j,:] = (z-z_hat).flatten()
            Sigma_IN[i,j,:,:] = np.dot(np.dot(H_hat_i, P_est), np.transpose(H_hat_i)) + R[j,:,:]

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

    Z = np.transpose(np.array(Z_temp)) # Z.shape = 2xk
    R = np.array(R_temp) # R.shape = kx2x2

# Callback funkcija ugradjeni /odom topic 
def odom_callback(data): 

    global X_odom, P_odom, U, b

    covariance = data.pose.covariance

    P_odom_temp = np.array(covariance)

    # indeksi koji su validni iz kovarijacione metrice 6x6 
    #                x     y     z  theta_x theta_y theta_z 
    #       x ->  [ (0),  (1),   2,    3,      4,    (5)]
    #       y ->  [ (6),  (7),   8,    9,     10,   (11)]
    #       z ->  [  12,   13,  14,   15,     16,     17]
    # Theta_x ->  [  18,   19,  20,   21,     22,     23]
    # Theta_y ->  [  24,   25,  26,   27,     28,     29]
    # Theta_z ->  [(30), (31),  32,   33,     34,   (35)]

    idx = [0,1,5,6,7,11,30,31,35]

    P_odom = P_odom_temp[idx].reshape((3,3))

    pose = data.pose.pose.position
    orient = data.pose.pose.orientation

    _,_,theta = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])

    X_odom = np.transpose(np.array([pose.x,pose.y,theta]))

def joint_callback(data):

    global sr_t, sl_t, wheel_diameter

    # predjeni put desnog tocka od pocetka rada programa
    sr_t = data.position[0]*0.5*wheel_diameter # konvezija u metre

    # predjeni put levog tocka od pocetka rada programa
    sl_t = data.position[1]*0.5*wheel_diameter # konvezija u metre

# -------------------------------GLAVNI PROGRAM----------------------------------------
#
# Glavni program za pokretanje svih gore navedenih funkcionalnosti
if __name__ == '__main__':

    try:
        # mapa
        M = loadMap()

        # b je rastojanje izmedju tockova robota
        b = 0.160 #[m]

        # wheel_diameter je precnik tockova robota 
        wheel_diameter = 0.066 #[m]

        # prag za Mahalonobisovu distancu
        g = 0.1
        

        Z = []
        R = []

        sr_last = 0
        sl_last = 0

        sl_t_1 = 0
        sr_t_1 = 0
        sr_t = 0
        sl_t = 0

        U = np.zeros((2,1))
        X_odom = np.zeros((3,1))
        P_odom = np.zeros((3,3))

        vel = Twist()
        pub_velocities = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

        rospy.init_node('robot_localization_EKF', anonymous=False, disable_signals=True)
        
        rospy.Subscriber('joint_states',JointState, joint_callback)
        rospy.Subscriber('odom', Odometry, odom_callback)
        rospy.Subscriber('line_segments', LineSegmentList, lidar_callback)

        pub_estimations = rospy.Publisher('position_estimations', String, queue_size = 1)

        X_final = np.zeros((3,1))
        P_final = P_odom

        while isinstance(Z, list) or isinstance(R, list):
            continue

        # prosireni Kalmanov Filtar
        while True:

            delta_sl = sl_t - sl_t_1
            delta_sr = sr_t - sr_t_1
            sl_t_1 = sl_t
            sr_t_1 = sr_t
            U = np.array([delta_sl, delta_sr]).reshape(2,1)

            X_prev = X_final
            P_prev = P_final

            X_est, P_est = getAprioriPrediction(X_prev, P_prev, U, b)
            V, H_est, R_est = associateMeasurement(X_est, P_est, Z, R, M, g)

            X_final, P_final = filterStep(X_est, P_est, V, H_est, R_est)
            
            print('--------------------------------------------------------')
            print('Estimirana pozicija: \n{} \nPozicija sa odometrija:\n{}\n'.format(X_final.flatten(), X_odom)) 
            print('Kovarijaciona matrica: \n{} \nKovarijaciona matrica sa odometrija:\n{}'.format(P_final, P_odom))
            print('--------------------------------------------------------')

            pub_estimations.publish(str(X_final[0,0]) + ' ' + str(X_final[1,0]) + ' ' + str(X_final[2,0]))

            time.sleep(1)

    except rospy.ROSInterruptException:
        pass
