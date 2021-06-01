#!/usr/bin/env python3

# Importovanje potrebnih biblioteka
import rospy
import math
from math import pi

# Importovanje potrebnih struktura poruka
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# -------------FUNKCIJE ZA REALIZACIJU AUTOMATSKOG UPRAVLJANJA BRZINA ROBOTA---------------
#
# Funkcija za publish-ovanje brzina na topic /cmd_vel 
def publish_velocities(velocities):

    global vel, pub

    vel.linear.x = velocities[0]
    vel.angular.z = velocities[1]
    pub.publish(vel)

# Funkcija za odredjivanje smera ugaone brzine prilikom automatskog kretanja
def w_sign(robot_angle, goal_angle):

    goal_angle = goal_angle if goal_angle>=0 else 2*pi+goal_angle
    robot_angle = robot_angle if robot_angle>=0 else 2*pi+robot_angle

    if(robot_angle < pi):
        if(goal_angle - robot_angle > pi):
            w = -1
        else:
            w = 1

        if(goal_angle < robot_angle):
            w = -1
    else:
        if(robot_angle-goal_angle > pi):
            w = 1
        else:
            w = -1

        if(goal_angle > robot_angle):
            w = 1

    return w

# Funkcija za automatsko kretanje robota sa zadatim ciljnim koordinatama
# iterativno se poziva u callback funkciji za dobijene estimirane pozicije
def automatic_control():

    global velocities, rho, theta, kr, kb, vw, dx, dy

    if rho > 0:

        val = - theta + math.atan2(dy, dx)

        if(min(abs(val), 2*pi-abs(val))>pi/2):
            reverse = True
        else:
            reverse = False
        
        alpha = val%(pi/2)
        beta = (- theta - alpha)%pi

        ka = max(2/pi*kr - 5/3*kb, (kr*rho/vw-kb*beta)/alpha)

        velocities[0] = kr*rho
        velocities[1] = (ka*alpha + kb*beta)*w_sign(theta, math.atan2(dy, dx))

        if(reverse):
            velocities[0] = -velocities[0]
            velocities[1] = -velocities[1]
        
        publish_velocities(velocities)

# -------------CALLBACK FUNKCIJE ZA DOHVATANJE PUTANJE I POZICIJA---------------
#
# Callback funkcija za dohvatanje ciljeva- kvadratne putanje 
def path_callback(data):

    global goals, goal_index

    path = data.data
    goal_index = 0
    goals = [float(el) for el in path.split(' ')]

# Callback funkcija za dohvatanje merenje trenutne pozicije 
# odnosno estimacija pomocu EKF-a i upravljanje robotom na osnovu tih 
# pozicija
def estimations_callback(data):

    global theta, rho, dx, dy, mod, goal_index, goals, velocities

    estimated_position = data.data
    
    position = [float(pos) for pos in estimated_position.split(' ')]

    xm = position[0]
    ym = position[1]
    theta = position[2]

    if (len(goals) != 0):

        if rho < 0.1:
            goal_index += 1

        if goal_index < 4:

            goal_x = goals[int(goal_index*2)]
            goal_y = goals[int(goal_index*2+1)]

            print('Trenutne koodinate cilja su: ', goal_x, goal_y)

            dx = goal_x - xm
            dy = goal_y - ym

            rho = math.sqrt(dx**2 + dy**2)

            automatic_control()

        else: 
            goals = []
            goal_index = 0
            velocities = [0, 0]

            publish_velocities(velocities)

# -------------GLAVNI DEO PROGRAMA ZA AUTOMATSKO UPRAVLJANJE ROBOTOM---------------
#
if __name__ == '__main__':
    try:
        vel = Twist()

        velocities = [0, 0]

        # trenutne izmerene pozicije robota - dobijene sa EKF-a
        xm, ym = 0, 0

        # Ciljne koordinate za automatsko kretanje robota
        goals = []
        goal_index = 0

        # Vrednosti medjurezultata pri racunanju automatskog upravljanja robota
        dx, dy = math.inf, math.inf 
        rho = math.inf

        # Izmerena trenutna orijentacija robota
        theta = 0

        # Parametri kontrolera (ka se racuna u toku kretanja)
        kr = 0.15
        kb = -0.05

        # Konstantni odnos v i w
        vw = 0.5

        # nod za kontrolu kretanja robota
        rospy.init_node('robot_kinematic_control', anonymous=False)

        # globalni publisher na topic /cmd_vel
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # globalni subscriberi na topic-e /path za citanje ciljeva u putanji robota
        # i za citanje trenutno merene pozicije sa EKF-a 
        rospy.Subscriber('path', String, path_callback)
        rospy.Subscriber('position_estimations', String, estimations_callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass