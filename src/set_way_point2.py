#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

# Variable globale pour stocker la pose de la tortue
current_pose = Pose()

def pose_callback(pose):
    global current_pose
    current_pose = pose
    pub = rospy.Publisher('set_way_point', waypoint)
def calculate_desired_angle(waypoint):
    dx = waypoint.x - current_pose.x
    dy = waypoint.y - current_pose.y
    desired_angle = math.atan2(dy, dx)
    return desired_angle

def calculate_error(desired_angle):
    error = math.atan2(math.sin(desired_angle - current_pose.theta), math.cos(desired_angle - current_pose.theta))
    return error

def regulation_control(error, kp):
    control = kp * error
    return control

def main():
    rospy.init_node('set_way_point')

    # Souscrire au topic "pose" pour obtenir la pose actuelle de la tortue
    rospy.Subscriber('pose', Pose, pose_callback)

    # Définir les coordonnées du waypoint
    waypoint = Pose()
    waypoint.x = 7
    waypoint.y = 7

    # Définir la constante Kp
    kp = 1.0  # Modifier cette valeur en fonction des tests

    # Créer un éditeur pour publier sur le topic "cmd_vel"
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10)  # Fréquence de publication (10 Hz)

    while not rospy.is_shutdown():
        # Calculer l'angle désiré
        desired_angle = calculate_desired_angle(waypoint)

        # Calculer l'erreur
        error = calculate_error(desired_angle)

        # Calculer la commande en cap
        control = regulation_control(error, kp)

        # Créer un message Twist pour la commande en vitesse angulaire
        cmd_vel = Twist()
        cmd_vel.angular.z = control

        # Publier le message
        cmd_vel_pub.publish(cmd_vel)

        rate.sleep()

if __name__ == '__main__':
    main()
