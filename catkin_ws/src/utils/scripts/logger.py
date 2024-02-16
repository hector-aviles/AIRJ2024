#!/usr/bin/env python3
"""
This node is a logger and records the following data:

"""
import beepy
import rospy
from std_msgs.msg import Float64MultiArray, Float64, Empty, Bool, String
from sensor_msgs.msg import Imu
from pathlib import Path
import os.path
from geometry_msgs.msg import Pose2D
from rosgraph_msgs.msg import Clock 

def callback_accelerometer(msg):

    global accel_x, accel_y, accel_z

    accel_x = msg.linear_acceleration.x
    accel_y = msg.linear_acceleration.y
    accel_z = msg.linear_acceleration.z    

def callback_accel_diff(msg):
    global accel_diff
    accel_diff = msg.data
    
def callback_action(msg):
    global action
    action = msg.data        

def callback_change_lane_finished(msg):
    global change_lane_finished
    change_lane_finished = msg.data
    
def callback_sim_time(msg):
    global curr_time            
    sim_time = msg
    sim_secs = sim_time.clock.secs 
    sim_nsecs = sim_time.clock.nsecs
    curr_time = sim_secs + sim_nsecs / (10**9)    

def callback_curr_lane(msg):
    global curr_lane
    curr_lane = msg.data    
    
def callback_cruise_enable(msg):
    global cruise_enable
    cruise_enable = msg.data        
    
def callback_left_lane(msg):
    global lane_rho_l, lane_theta_l
    lane_rho_l, lane_theta_l = msg.data

def callback_right_lane(msg):
    global lane_rho_r, lane_theta_r
    lane_rho_r, lane_theta_r = msg.data     
    
def callback_follow_enable(msg):
    global follow_enable
    callback_follow_enable = msg.data

def callback_free_east(msg):
    global free_east
    free_east = msg.data
    
def callback_free_north(msg):
    global free_north
    free_north = msg.data
    
def callback_free_north_east(msg):
    global free_north_east
    free_north_east = msg.data    
    
def callback_free_north_west(msg):
    global free_north_west
    free_north_west = msg.data

def callback_free_south_east(msg):
    global free_south_east
    free_south_east = msg.data

def callback_free_south_west(msg):
    global free_south_west
    free_south_west = msg.data

def callback_free_west(msg):
    global free_west
    free_west = msg.data

def callback_distance_to_north(msg):
    global distance_to_north
    distance_to_north = msg.data

def callback_pass_finished(msg):
    global pass_finished
    callback_pass_finished = msg.data    

def callback_curr_pos(msg):
    global sdc_curr_pos   

    sdc_curr_pos.x = msg.x
    sdc_curr_pos.y = msg.y
    sdc_curr_pos.theta = msg.theta
        
def callback_speed(msg):
    global speed
    speed = msg.data 
    
def callback_start_change_lane_on_left(msg):
    global start_change_lane_on_left
    start_change_lane_on_left = msg.data

def callback_start_change_lane_on_right(msg):
    global start_change_lane_on_right
    start_change_lane_on_right = msg.data
    
def callback_steering(msg):
    global steering
    steering = msg.data      

def callback_success(msg):
    global success
    success = msg.data
    
def callback_goal_reached(msg):
    global goal_reached
    goal_reached = msg.data    
       
def callback_car_1_pos(msg):
    global car_1_pos   

    car_1_pos.x = msg.x
    car_1_pos.y = msg.y
    car_1_pos.theta = msg.theta       
       
def callback_car_2_pos(msg):
    global car_2_pos   

    car_2_pos.x = msg.x
    car_2_pos.y = msg.y
    car_2_pos.theta = msg.theta              
           
def callback_car_3_pos(msg):
    global car_3_pos   

    car_3_pos.x = msg.x
    car_3_pos.y = msg.y
    car_3_pos.theta = msg.theta           
         
def callback_car_4_pos(msg):
    global car_4_pos   

    car_4_pos.x = msg.x
    car_4_pos.y = msg.y
    car_4_pos.theta = msg.theta
    
def callback_car_5_pos(msg):
    global car_5_pos   

    car_5_pos.x = msg.x
    car_5_pos.y = msg.y
    car_5_pos.theta = msg.theta    

def callback_car_6_pos(msg):
    global car_6_pos   

    car_6_pos.x = msg.x
    car_6_pos.y = msg.y
    car_6_pos.theta = msg.theta
    
def callback_car_7_pos(msg):
    global car_7_pos   

    car_7_pos.x = msg.x
    car_7_pos.y = msg.y
    car_7_pos.theta = msg.theta
    
def callback_car_8_pos(msg):
    global car_8_pos   

    car_8_pos.x = msg.x
    car_8_pos.y = msg.y
    car_8_pos.theta = msg.theta
    
def callback_car_9_pos(msg):
    global car_9_pos   

    car_9_pos.x = msg.x
    car_9_pos.y = msg.y
    car_9_pos.theta = msg.theta

def callback_car_10_pos(msg):
    global car_10_pos   

    car_10_pos.x = msg.x
    car_10_pos.y = msg.y
    car_10_pos.theta = msg.theta                        
               
def main():
    global cruise_enable
    global accel_x, accel_y, accel_z, accel_diff, action, change_lane_finished, curr_time, curr_lane, lane_rho_l, lane_theta_l, lane_rho_r, lane_theta_r, follow_enable, free_east, free_north, free_north_east, free_north_west, free_south_east, free_south_west, free_west, distance_to_north, pass_finished, sdc_curr_pos, speed, start_change_lane_on_left, start_change_lane_on_right, steering, success, goal_reached, car_1_pos, car_2_pos, car_3_pos, car_4_pos, car_5_pos, car_6_pos, car_7_pos, car_8_pos, car_9_pos, car_10_pos
    
    sdc_curr_pos = Pose2D()   
    car_1_pos = Pose2D() 
    car_2_pos = Pose2D() 
    car_3_pos = Pose2D()                
    car_4_pos = Pose2D() 
    car_5_pos = Pose2D() 
    car_6_pos = Pose2D() 
    car_7_pos = Pose2D()
    car_8_pos = Pose2D() 
    car_9_pos = Pose2D() 
    car_10_pos = Pose2D()
    

    # 31 variables
    accel_x = 0.0
    accel_y = 0.0 
    accel_z = 0.0 
    accel_diff = 0.0 
    action = "" 
    change_lane_finished = False 
    curr_time = 0.0 
    curr_lane = True 
    lane_rho_l = 0.0 
    lane_theta_l = 0.0 
    lane_rho_r = 0.0 
    lane_theta_r = 0.0 
    follow_enable = False 
    free_east = True 
    free_north = True 
    free_north_east = True 
    free_north_west = True 
    free_south_east = True 
    free_south_west = True 
    free_west = True 
    distance_to_north = 0.0 
    pass_finished = False 
    sdc_curr_pos.x = sdc_curr_pos.y = sdc_curr_pos.theta = 0.0 
    speed = 0.0 
    start_change_lane_on_left = False
    start_change_lane_on_right = False 
    steering = 0.0 
    success = True    
    goal_reached = False        
    
    # Extras
    car_1_pos.x = car_1_pos.y = car_1_pos.theta = 0.0     
    car_2_pos.x = car_2_pos.y = car_2_pos.theta = 0.0
    car_3_pos.x = car_3_pos.y = car_3_pos.theta = 0.0
    car_4_pos.x = car_4_pos.y = car_4_pos.theta = 0.0
    car_5_pos.x = car_5_pos.y = car_5_pos.theta = 0.0
    car_6_pos.x = car_6_pos.y = car_6_pos.theta = 0.0     
    car_7_pos.x = car_7_pos.y = car_7_pos.theta = 0.0
    car_8_pos.x = car_8_pos.y = car_8_pos.theta = 0.0
    car_9_pos.x = car_9_pos.y = car_9_pos.theta = 0.0
    car_10_pos.x = car_10_pos.y = car_10_pos.theta = 0.0    
                
    num_trials_file = ".trial_number.data"

    print("INITIALIZING LOGGER...", flush = True)
    rospy.init_node("logger")
    rate = rospy.Rate(5)     
        
    rospy.Subscriber('/accelerometer', Imu, callback_accelerometer)    
    rospy.Subscriber("/accelerometer_diff", Float64, callback_accel_diff)
    rospy.Subscriber("/action", String, callback_action)    
    rospy.Subscriber("/change_lane_finished", Bool, callback_change_lane_finished)
    rospy.Subscriber("/clock", Clock, callback_sim_time) 
    rospy.Subscriber("/cruise/enable", Bool, callback_cruise_enable)
    rospy.Subscriber("/current_lane", Bool, callback_curr_lane)
    rospy.Subscriber("/demo/left_lane" , Float64MultiArray, callback_left_lane)
    rospy.Subscriber("/demo/right_lane", Float64MultiArray, callback_right_lane)
    rospy.Subscriber("/follow/enable", Bool, callback_follow_enable)    
    rospy.Subscriber("/free/east"     , Bool, callback_free_east)
    rospy.Subscriber("/free/north"     , Bool, callback_free_north)
    rospy.Subscriber("/free/north_east"     , Bool, callback_free_north_east)
    rospy.Subscriber("/free/north_west", Bool, callback_free_north_west)
    rospy.Subscriber("/free/south_east", Bool, callback_free_south_east)
    rospy.Subscriber("/free/south_west", Bool, callback_free_south_west)
    rospy.Subscriber("/free/west"      , Bool, callback_free_west)
    rospy.Subscriber("/obstacle/distance"  , Float64, callback_distance_to_north)
    rospy.Subscriber("/pass_finished", Bool, callback_pass_finished)
    rospy.Subscriber("/self_driving_pose", Pose2D, callback_curr_pos) 
    rospy.Subscriber("/speed", Float64, callback_speed)
    rospy.Subscriber("/start_change_lane_on_left", Bool, callback_start_change_lane_on_left)
    rospy.Subscriber("/start_change_lane_on_right", Bool, callback_start_change_lane_on_right)
    rospy.Subscriber("/steering", Float64, callback_steering)
    rospy.Subscriber("/success", Bool, callback_success)    
    rospy.Subscriber("/goal_reached", Bool, callback_goal_reached)        
    
    # Extras
    rospy.Subscriber("/car_1_pose", Pose2D, callback_car_1_pos)
    rospy.Subscriber("/car_2_pose", Pose2D, callback_car_2_pos)    
    rospy.Subscriber("/car_3_pose", Pose2D, callback_car_3_pos)
    rospy.Subscriber("/car_4_pose", Pose2D, callback_car_4_pos)
    rospy.Subscriber("/car_5_pose", Pose2D, callback_car_5_pos)
    rospy.Subscriber("/car_6_pose", Pose2D, callback_car_6_pos)        
    rospy.Subscriber("/car_7_pose", Pose2D, callback_car_7_pos)
    rospy.Subscriber("/car_8_pose", Pose2D, callback_car_8_pos)
    rospy.Subscriber("/car_9_pose", Pose2D, callback_car_9_pos)
    rospy.Subscriber("/car_10_pose", Pose2D, callback_car_10_pos)    
   
    # Lectura del número de repetición
    print ("Reading the number of the trial...", flush = True, end="")
    write_header_csv = False
    file_exists = os.path.exists(num_trials_file)
    if file_exists:
       c = open(num_trials_file, "r")
       repetition = c.read()
       c.close()
       trial_number = int(repetition)
    else:
       myfile = Path(num_trials_file)
       myfile.touch(exist_ok = True)
       c = open(num_trials_file, "w")
       trial_number = 1                
       c.write(str(trial_number))
       c.close()

    print (" Trial number", trial_number, flush = True)       
    if trial_number == 1:
       write_header_csv = True    

    print ("Opening logfile...", flush = True, end="")    
    logfile = ".logfile.csv"
    f = open(logfile,"a")
    print (" Done.", flush = True)    
         
    print("Logger.->Waiting for start signal")
    rospy.wait_for_message("/policy_started", Empty, timeout=10000.0)
    print("Logger.->Start signal received")
    
    # Pause the logger 
    rate = rospy.Rate(1) #Hz
    rate.sleep()
    rate = rospy.Rate(2) #Hz    

    iteration = 1
    while not rospy.is_shutdown():
    
        if write_header_csv == True:
           output = "trial_num," + "iteration," + "accel_x," + "accel_y," + "accel_z," + "accel_diff," + "action," + "change_lane_finished," + "curr_time," + "curr_lane," + "lane_rho_l," + "lane_theta_l," + "lane_rho_r," + "lane_theta_r," + "follow_enable," + "free_east," + "free_north," + "free_north_east," + "free_north_west," + "free_south_east," + "free_south_west," + "free_west," + "distance_to_north," + "pass_finished," + "sdc_curr_pos.x," + "sdc_curr_pos.y," + "sdc_curr_pos.theta," + "speed," + "start_change_lane_on_left," + "start_change_lane_on_right," + "steering," + "success," + "goal_reached,"+ "car_1_pose.x," + "car_1_pose.y," + "car_2_pose.x," + "car_2_pose.y," + "car_3_pose.x," + "car_3_pose.y," +  "car_4_pose.x," + "car_4_pose.y," +  "car_5_pose.x," + "car_5_pose.y," + "car_6_pose.x," + "car_6_pose.y," + "car_7_pose.x," + "car_7_pose.y," + "car_8_pose.x," + "car_8_pose.y," + "car_9_pose.x," + "car_9_pose.y," + "car_10_pose.x," + "car_10_pose.y" +  "\n"
           
           write_header_csv = False  # Write the header of the csv only once   

           f.write(output)
    
        now = rospy.get_time()

        output = str(trial_number) + "," + str(iteration) + "," + str(accel_x) + "," + str(accel_y) + "," + str(accel_z) + "," + str(accel_diff) + "," + str(action) + "," + str(change_lane_finished) + "," + str(curr_time) + ","  +  str(curr_lane) + "," + str(lane_rho_l) + "," + str(lane_theta_l) + "," +  str(lane_rho_r) + "," + str(lane_theta_r) + "," + str(follow_enable) + "," +  str(free_east) + "," + str(free_north) + "," + str(free_north_east) + "," + str(free_north_west) + "," + str(free_south_east) + "," +  str(free_south_west) + "," + str(free_west) + "," + str(distance_to_north) + "," + str(pass_finished) + "," + str(sdc_curr_pos.x) + "," +  str(sdc_curr_pos.y) + "," + str(sdc_curr_pos.theta) + "," + str(speed) + ","  +  str(start_change_lane_on_left) + "," + str(start_change_lane_on_right) + "," +  str(steering) + "," + str(success) + "," + str(goal_reached) + "," + str(car_1_pos.x) + "," + str(car_1_pos.y) + "," + str(car_2_pos.x) + ","  + str(car_2_pos.y) + ","  + str(car_3_pos.x) + "," + str(car_3_pos.y) + "," + str(car_4_pos.x) + "," + str(car_4_pos.y) + "," + str(car_5_pos.x) + "," + str(car_5_pos.y) + "," + str(car_6_pos.x) + "," + str(car_6_pos.y) + "," + str(car_7_pos.x) + ","  + str(car_7_pos.y) + ","  + str(car_8_pos.x) + "," + str(car_8_pos.y) + "," + str(car_9_pos.x) + "," + str(car_9_pos.y) + "," + str(car_10_pos.x) + "," + str(car_10_pos.y) +  "\n"
                       
        f.write(output) 
        
        # Break the loop and write current data
        if success == False or goal_reached == True:        
           if success == False:
              print ("Something weird happened", flush = True)
           if goal_reached == True:
              print ("Well done!", flush = True)
           
           break            
           
        rate.sleep()      
        iteration = iteration + 1   
        
    print ("Closing logfile...", flush = True, end="")
    f.close()
    print (" Done.", flush = True)

    print ("Updating trial number...", flush = True, end="")        
    c = open(num_trials_file, "w")        
    trial_number = trial_number + 1
    c.write(str(trial_number))
    c.close()
    print (" Done.", flush = True) 
    
    beepy.beep(sound="coin")
        
             
if __name__ == "__main__":
    try:
        main()
                        
    except:
        rospy.ROSInterruptException
        pass

    
