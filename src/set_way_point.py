#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import sqrt

# Variable globale pour stocker la pose de la tortue
turtle_pose = Pose()

# Variable globale pour le waypoint
waypoint = Pose()
waypoint.x = 7
waypoint.y = 7

def pose_callback(msg):
    global turtle_pose
    turtle_pose = msg

def calculate_distance(p1, p2):
    return sqrt((p2.y - p1.y)**2 + (p2.x - p1.x)**2)

def main():
    # Initialiser le nœud
    rospy.init_node('distance_regulation')

    # Souscrire au topic 'pose' pour obtenir la pose de la tortue
    rospy.Subscriber('pose', Pose, pose_callback)

    # Créer un éditeur pour publier sur le topic 'cmd_vel'
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    # Créer un éditeur pour publier sur le topic 'is_moving'
    is_moving_pub = rospy.Publisher('is_moving', Bool, queue_size=1)

    # Paramètres
    Kpl = rospy.get_param('~Kpl', 1.0)
    distance_tolerance = rospy.get_param('~distance_tolerance', 0.1)

    # Fréquence de publication (en Hz)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Calculer la distance entre le waypoint et la position de la tortue
        distance = calculate_distance(waypoint, turtle_pose)

        # Vérifier si la distance est supérieure à la distance_tolerance
        if distance > distance_tolerance:
            # Calculer la commande linéaire
            linear_error = distance
            linear_velocity = Kpl * linear_error

            # Publier la commande linéaire sur le topic 'cmd_vel'
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = linear_velocity
            cmd_vel_pub.publish(cmd_vel_msg)

            # Publier True sur le topic 'is_moving'
            is_moving_msg = Bool()
            is_moving_msg.data = True
            is_moving_pub.publish(is_moving_msg)
        else:
            # Publier False sur le topic 'is_moving'
            is_moving_msg = Bool()
            is_moving_msg.data = False
            is_moving_pub.publish(is_moving_msg)

        rate.sleep()

if _name_ == '_main_':
    main()
