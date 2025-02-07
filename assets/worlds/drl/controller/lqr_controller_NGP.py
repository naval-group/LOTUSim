#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
import numpy as np
import control
from std_msgs.msg import Float64
import time
from stable_baselines3 import SAC
from rclpy.node import Node
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Pose
import signal
import sys
import matplotlib.pyplot as plt
import datetime
import os
import pathlib


# try first to source the environment to load this message, don't forget to do that  
# from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped 

def signal_handler(sig, frame):
    print('Interruption détectée. Sauvegarde des courbes...')
    save_curveserrors(historique_x, historique_y, historique_z, historique_yaw)

    sys.exit(0)

# Configuration du gestionnaire de signaux
signal.signal(signal.SIGINT, signal_handler)

def save_curveserrors(historique_x, historique_y, historique_z, historique_yaw):
    variables = {
        "historique_x": historique_x,
        "historique_y": historique_y,
        "historique_z": historique_z,
        "historique_yaw": historique_yaw
    }
    
    current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    date_folder = os.path.join(directory_path, 'result/',current_time)
    print(date_folder)
    os.makedirs(date_folder)
    for variable_name, data in variables.items():
        plt.figure()
        plt.plot(np.linspace(0,len(historique_x), len(historique_x)), data)
        plt.title(variable_name)
        plt.xlabel('step')
        plt.ylabel(variable_name)
       
     
        file_path = os.path.join(date_folder, variable_name)
        plt.savefig(file_path)
        plt.close()






def quaternion_to_euler(x, y, z, w):
    """
    Convert quaternion to Euler angles.

    Args:
        x, y, z, w: Components of the quaternion.

    Returns:
        phi, theta, psi: Euler angles representing roll, pitch, and yaw.
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    phi = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if np.abs(sinp) >= 1:
        theta = np.sign(sinp) * np.pi / 2
    else:
        theta = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    psi = np.arctan2(siny_cosp, cosy_cosp)

    return phi, theta, psi



TAM = np.array([[0.7071, 0.7071, -0.7071, -0.7071, 0.0, 0.0, 0.0, 0.0],[-0.7071, 0.7071, -0.7071, 0.7071, 0.0, 0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 1.0, -1.0],[0.0601, -0.0601, 0.0601, -0.0601, -0.218, -0.218, 0.218, 0.218],[0.0601,0.0601, -0.0601, -0.0601, 0.12, -0.12, 0.12, -0.12],[-0.1881, 0.1881, 0.1881, -0.1881, 0.0, 0.0, 0.0, 0.0]])

#TAM[0][3]*= 2



ref_x = 4
ref_y = 0
ref_z = -2.5

ref_psi = 0.0 
ref_phi=0
ref_theta=0
previous_error=0
compteur = 0
K_previous = 0
A_constant = 0
applied_TAM = TAM.copy()
directory_path = pathlib.Path(__file__).parent.resolve()
model_path = os.path.join(directory_path,"models/SAC")
model = SAC.load(model_path)
lqr_launch = False
integral_term_pid = np.zeros(shape=(6,))
error_pose_pid = np.zeros(shape=(6,))
dt = 0.04
compteur_pid = 0
end_lqr = False





def odometry_callback(msg):
    global lqr_launch
    global compteur_pid
    global end_lqr
    quaternion = (
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w
    )
    euler = quaternion_to_euler(*quaternion)
    phi= euler[0]  
    theta=euler[1]
    psi = euler[2]  
    u = msg.twist.twist.linear.x
    v = msg.twist.twist.linear.y
    w = msg.twist.twist.linear.z
    p = msg.twist.twist.angular.x
    q = msg.twist.twist.angular.y
    r = msg.twist.twist.angular.z

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    print("observation", x, y, z)
    #lqr_control(msg)
    
    if end_lqr == False:
        if -2.6<z<-2.4 and -0.1<x<0.1 and -0.1<y<0.1 and lqr_launch == False:

            lqr_control(msg, False, False)
            lqr_launch = True

        elif lqr_launch:
            lqr_control(msg, False, False) 

        else:
            ref_pid = [0, 0, -2.5, 0, 0, 0]
            lqr_control(msg, True, False)
        if x > 1.5:
            end_lqr = True
    else:
        ref_pid = [1.5, 0, 0, 0, 0, 0]
        lqr_control(msg, True, True)
    

def pid_control(msg, ref_pid):
    global integral_term_pid
    global error_pose_pid

    quaternion = (
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w
    )
    euler = quaternion_to_euler(*quaternion)
    phi= euler[0]  
    theta=euler[1]
    psi = euler[2]  
    u = msg.twist.twist.linear.x
    v = msg.twist.twist.linear.y
    w = msg.twist.twist.linear.z
    p = msg.twist.twist.angular.x
    q = msg.twist.twist.angular.y
    r = msg.twist.twist.angular.z

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z

    Kp = np.diag((10, 10, 20, 20, 40, 10))
    Kd = np.diag((10, 10, 20,10, 20, 10))
    Ki = np.diag((1, 1, 1, 1,1, 1))


    
    error_pose_euler = np.array([ref_pid[0]-x, ref_pid[1]-y, ref_pid[2]-z, ref_pid[3]-phi, ref_pid[4]-theta, ref_pid[5]-psi])
    error_vel = np.array([0-u, 0-v, 0-w, 0-p, 0-q, 0-r])
    integral_term_pid = integral_term_pid  + 0.5 * (error_pose_euler + integral_term_pid) * dt
    # Store the current pose error for the next iteration
    integral_term_pid = error_pose_euler
    tau = np.dot(Kp, error_pose_euler) + np.dot(Kd, error_vel) + np.dot(Ki, integral_term_pid )

    control_input = np.linalg.pinv(TAM).dot(tau) 
    #print("control input", control_input)
    #print("forces", tau)
    #print("time", time.time())
    thruster_msg = Float64() 
    print("control_input", control_input)
    for i in range(8):
        thruster_msg = Float64()  # Create  FloatStamped message to use it to send the thruster forces
        #thruster_msg.header.stamp = rospy.Time.now()  # Timestamp the message
        thruster_msg.data = control_input[i]  # Assign the thrust force value
        pub_thrusters[i].publish(thruster_msg)  # Publish the thrust forces

temps = 0
t_previous = 0
integral_term = np.zeros(shape=(6,))
error_pose = np.zeros(shape=(6,))
dt = 0.04

Kp = np.diag((90, 90, 90, 4,4, 10))
Ki = np.diag((5, 5, 15, 0.5,1, 0.5))
Kd = np.diag((100, 100, 80,0.3, 0.3, 0.1))


updated_TAM = TAM.copy()
compteur = 0

B_new = np.array([
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0.0518824 , 0, 0, 0, 0, 0],
        [0, 0.041322314, 0, 0, 0, 0],
        [0, 0, 0.038358266, 0, 0, 0],
        [0, 0, 0, 3.57142857 , 0, 0],
        [0, 0, 0, 0, 3.57142857, 0],
        [0, 0, 0, 0, 0, 3.57142857] 
        ])

A = np.array([
    [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, -3.75106165e-07,
     3.04880378e-02, 5.86034060e-07, 9.99999795e-01, 1.23749533e-05,
     -6.40492217e-04, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00],
    [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, -3.01723212e-02,
     -3.75481593e-07, 4.92917503e-01, -1.23156996e-05, 9.99999996e-01,
     9.25166185e-05, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00],
    [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 5.48458998e-06,
     -4.92917503e-01, 0.00000000e+00, 6.40493359e-04, -9.25087115e-05,
     9.99999791e-01, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00],
    [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 7.75337841e-07,
     -1.11716408e-05, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
     0.00000000e+00, 1.00000000e+00, 5.92512396e-08, -6.40493487e-04],
    [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.11716362e-05,
     0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
     0.00000000e+00, 0.00000000e+00, 9.99999996e-01, 9.25087304e-05],
    [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, -1.21053221e-03,
     7.15536174e-09, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
     0.00000000e+00, 0.00000000e+00, -9.25087494e-05, 1.00000020e+00],
    [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
     0.00000000e+00, 0.00000000e+00, -1.29188503e+00, 7.96623639e-07,
     -2.19106097e-04, 0.00000000e+00, 5.46119020e-03, -1.90153189e-07],
    [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
     0.00000000e+00, 0.00000000e+00, 2.79833799e-06, -2.57004821e-01,
     8.58755626e-06, -3.83188484e-03, 0.00000000e+00, -1.22248358e-01],
    [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
     0.00000000e+00, 0.00000000e+00, -2.78422112e-04, -3.11045345e-06,
     -2.84689393e-01, 1.23895846e-07, 1.13375494e-01, 0.00000000e+00],
    [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
     0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 2.01551108e-01,
     1.79918315e-05, -2.49924943e-01, 0.00000000e+00, 0.00000000e+00],
    [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
     0.00000000e+00, 0.00000000e+00, -9.77583218e-01, 0.00000000e+00,
     -1.59711565e+01, 0.00000000e+00, -1.91656311e-01, 0.00000000e+00],
    [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
     0.00000000e+00, 0.00000000e+00, 6.92200704e-05, 1.26684791e+01,
     0.00000000e+00, 0.00000000e+00, 0.00000000e+00, -2.49987475e-01]
])

historique_x = []
historique_y = []
historique_z = []
historique_yaw = []

def lqr_control(msg, use_pid, use_pid_end):

    global integral_term
    global error_pose
    global updated_TAM
    global compteur
    global historique_x, historique_y, historique_z, historique_yaw
    compteur += 1
    data = msg
    
        
    position = data.pose.pose.position
    orientation = data.pose.pose.orientation

    linear_speed = data.twist.twist.linear
    angular_speed = data.twist.twist.angular
    u, v, w = linear_speed.x, linear_speed.y, linear_speed.z
    p, q, r = angular_speed.x, angular_speed.y, angular_speed.z
    roll, pitch, yaw = quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)

    phi, theta, psi = roll, pitch, yaw

       
    error_pose_euler = np.array([2-position.x, (0-position.y), (-2.5-position.z), 0-roll, (0-pitch)*5, (0-yaw)*8])
    if use_pid:
        error_pose_euler = np.array([0-position.x, (0-position.y), (-2.5-position.z), 0-roll, (0-pitch)*5, (0-yaw)*8])   

    if use_pid_end:
        error_pose_euler = np.array([1.5-position.x, (0-position.y), (0-position.z), 0-roll, (0-pitch)*5, (0-yaw)*8])  
    error_vel = np.array([0-linear_speed.x, 0-linear_speed.y, 0-linear_speed.z, 0-angular_speed.x, 0-angular_speed.y, 0-angular_speed.z])
    limits_positive_int = [1, 1, 1, 1, 1, 1]
    limits_negative_int = [-1, -1, -1, -1, -1, -1]
    integral_term = integral_term + 0.5 * (error_pose_euler + error_pose) * dt
    for i in range(len(integral_term)):
        if integral_term[i] > limits_positive_int[i]:
            integral_term[i] = limits_positive_int[i]
        elif integral_term[i] < limits_negative_int[i]:
            integral_term[i] = limits_negative_int[i]

    error_pose = error_pose_euler
    tau_vertical = np.dot(Kp, error_pose_euler) + np.dot(Kd, error_vel) + np.dot(Ki, integral_term)
    tau_vertical[2] = -tau_vertical[2] #z
    tau_vertical[1] = -tau_vertical[1] #y
    tau_vertical[4] = -tau_vertical[4] #pitch
    tau_vertical[5] = -tau_vertical[5] #yaw
    limits_positive = [30, 30, 30, 0.1, 0.1, 3]
    limits_negative = [-30, -30, -30, -0.1, -0.1, -3]
    for i in range(len(tau_vertical)):
        if tau_vertical[i] > limits_positive[i]:
            tau_vertical[i] = limits_positive[i]
        elif tau_vertical[i] < limits_negative[i]:
            tau_vertical[i] = limits_negative[i]

    control_input_vertical = np.linalg.pinv(TAM).dot(tau_vertical) 


    # the full states are:
    full_state = np.array([position.x, position.y, position.z, phi, theta, psi, 0, 0,0,0,0,0])
    if not use_pid_end:
        historique_x.append(position.x)
        historique_y.append(position.y)
        historique_z.append(position.z)
        historique_yaw.append(psi)
    sigma1 = np.sin(phi) * np.sin(psi) + np.cos(phi) * np.cos(psi) * np.sin(theta)
    sigma2 = np.cos(phi) * np.cos(psi) + np.sin(phi) * np.sin(psi) * np.sin(theta)
    sigma3 = np.cos(psi) * np.sin(phi) - np.cos(phi) * np.sin(psi) * np.sin(theta)
    sigma4 = np.cos(phi) * np.sin(psi) - np.cos(psi) * np.sin(phi) * np.sin(theta)
    sigma5 = np.tan(theta)**2 + 1.0


  

    
    if compteur%5 == 0 and compteur > 20:
        observation = [round((0-position.y)*1.1, 2), round((0-linear_speed.y)*2.3, 2), round((0-yaw)*1.3*1.8, 2), round((0-angular_speed.z)*1.3*1.8, 2)]
        action, _ = model.predict(observation, deterministic=True)

        action = [x / 10. for x in action]
        coeffs=[1.25-10.5*action[0],1.25-10.5*action[1],1.25-10.5*action[2],1.25-10.5*action[3]]

        updated_TAM = TAM.copy()
        updated_TAM[0][0] = TAM[0][1]*(1/coeffs[0])
        updated_TAM[0][1] = TAM[0][1]*(1/coeffs[1])
        updated_TAM[0][2] = TAM[0][2]*(1/coeffs[2])
        updated_TAM[0][3] = TAM[0][3]*(1/coeffs[3])
 
        print("updated TAM: ", updated_TAM)
    
    #B=np.dot(B_new,TAM)  #sans drl
   
    if drl_activate:
        B=np.dot(B_new,updated_TAM) #avec drl
    else: 
        B=np.dot(B_new,TAM)
 



    Q = np.diag([13, 23, 28, 10, 10, 13, 1, 1,1,1,1,1]) #all 10 working
    R = np.diag([ 1, 1, 1, 1, 1, 1, 1, 1])




            
    K, _, _ = control.lqr(A, B, Q, R)  # Compute the LQR gain matrix K

       
    full_reference = np.array([ref_x, ref_y, ref_z, ref_phi, ref_theta,ref_psi, 0, 0,0,0,0,0])  
    error = full_state - full_reference

    error[5] = -error[5]*30

 
    control_input = -np.dot(K, error) #thrusts
    if fault_activate:
        print("yes")
        control_input[0] = control_input[0]*0 #fault application
    tau = TAM.dot(control_input) #forces
    tau[2] = tau_vertical[2]
    tau[3] = 0
    tau[4] = 0
    
    if use_pid:
        tau = tau_vertical       
    limits_positive = [30, 30, 30, 0.1, 0.1, 5]
    limits_negative = [-30, -30, -30, -0.1, -0.1, -5]
    for i in range(len(tau_vertical)):
        if tau[i] > limits_positive[i]:
            tau[i] = limits_positive[i]
        elif tau[i] < limits_negative[i]:
            tau[i] = limits_negative[i]
    #tau = np.array([0, 0, 20, 0, 0, 0]) #TEST
    tau[0] *= 0.1
    print("tau: ", tau)
    control_input = np.linalg.pinv(TAM).dot(tau) #thrusts
    #control_input[6]=20
    #control_input[7]=-20
    #control_input[4]=0
    #control_input[5]=0
    print("control input:", control_input)
    for i in range(8):
        thruster_msg = Float64()  # Create  FloatStamped message to use it to send the thruster forces
        #thruster_msg.header.stamp = rospy.Time.now()  # Timestamp the message
        thruster_msg.data = control_input[i]  # Assign the thrust force value
        pub_thrusters[i].publish(thruster_msg)  # Publish the thrust forces
    

if __name__ == '__main__':

     
    fault_activate = input('Activate fault on thruster O? (y/n):').lower().strip() == 'y'      
    drl_activate = input('Activate DRL? (y/n):').lower().strip() == 'y'
    

    

    rclpy.init()
    node = Node('lqr_control_node')
    #rospy.init_node('lqr_control_node')

    # Define the publisher and subscriber
    #rospy.Subscriber('/bluerov/pose_gt', Odometry, odometry_callback)
    node.create_subscription(Odometry, '/model/bluerov2_heavy/odometry', odometry_callback,10)
    pub_thrusters = []
    
    for i in range(8):
        pub_thrusters.append(node.create_publisher(Float64, f'/bluerov2_heavy/thruster{i+1}/cmd_thrust', 10))
        

    # Keep the node running
    rclpy.spin(node)

