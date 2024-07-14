#! /usr/bin/env python
# -*- coding: utf-8 -*-
import os
import rospy
import tf
import time
import sys
import math
import kachaka_api
import threading
from PIL import Image
import io
import numpy as np
import cv2
import yaml
import subprocess
import actionlib
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from ros_msgs.srv import StringTrigger, StringTriggerResponse


__dir = os.path.dirname(os.path.abspath(__file__))

def sending_thread():
    while True:
        pose = client.get_robot_pose()
        br.sendTransform((pose.x, pose.y, 0),
                        tf.transformations.quaternion_from_euler(0, 0, pose.theta ),
                        rospy.Time.now(),
                        "base_footprint",
                        "map")

        time.sleep(0.1)

def save_map( base_name ):
    png_map = client.get_png_map()
    png_map_img = np.array( Image.open(io.BytesIO(png_map.data)) )

    gray_map = np.zeros( png_map_img.shape[:2], dtype=np.uint8 ) 
    gray_map[:,:] = 127
    gray_map[ png_map_img[:,:,0]>250 ] = 255
    gray_map[ png_map_img[:,:,0]<200 ] = 0 
    cv2.imwrite( base_name + ".pgm", gray_map )

    ori_x = png_map.origin.x
    ori_y = png_map.origin.y
    map_info = {}
    map_info["image"] = base_name + ".pgm"
    map_info["resolution"] = png_map.resolution
    map_info["origin"] = [ ori_x, ori_y, 0 ]
    map_info["negate"] = 0
    map_info["occupied_thresh"] = 0.65
    map_info["free_thresh"] = 0.196

    with open( base_name + ".yaml","w")as f:
        yaml.dump(map_info, f)

    return gray_map, map_info

def publish_map():
    img, map_info = save_map( os.path.join( __dir, "map") )

    map = OccupancyGrid()
    map.header.stamp = rospy.Time.now()
    map.header.frame_id = "map"
    map.info.resolution = map_info["resolution"]
    map.info.width = img.shape[1]
    map.info.height = img.shape[0]
    map.info.origin.position.x = map_info["origin"][0]
    map.info.origin.position.y = map_info["origin"][1]
    map.data = np.flip(img,0).reshape(-1).astype(np.int8)

    pub = rospy.Publisher("/map", OccupancyGrid, queue_size=10, latch=True)
    pub.publish(map)


def move_base_action_server( goal ):
    if type(goal) is MoveBaseGoal:
        target_pose = goal.target_pose
    else:
        target_pose = goal

    x = target_pose.pose.position.x
    y = target_pose.pose.position.y

    rot_x = target_pose.pose.orientation.x 
    rot_y = target_pose.pose.orientation.y
    rot_z = target_pose.pose.orientation.z 
    rot_w = target_pose.pose.orientation.w

    theta = tf.transformations.euler_from_quaternion((rot_x, rot_y, rot_z, rot_w))[2]

    print(f"[{x}, {y}, {theta}]へ移動")
    client.move_to_pose(x, y, theta)

def speak( data ):
    client.speak(data.data)


def check_types( com, args, types ):
    if len(args)!=len(types):
        return None, f"{com} takes {len(types)} but {len(args)} were given. "
    
    try:
        converted_args = [ t(a) for a,t in zip(args,types) ]
        return converted_args, ""
    except:
        return None, f"{com} requires {types} but {args} were given. "

# kachaka apiのこの関数がなぜか動かないので独自実装
def rotate_in_place( delta_theta ):
    client.set_manual_control_enabled(True)
    init_theta = client.get_robot_pose().theta

    delta_theta = normalize_angle(delta_theta)

    while 1:
        cur_theta = client.get_robot_pose().theta
        diff_theta = cur_theta-init_theta
        diff_theta = abs(normalize_angle( diff_theta ))

        rot_speed = np.clip( (abs(delta_theta)-diff_theta)*2 , 0.3, 1.5)
        if delta_theta<0:
            rot_speed *=-1

        print(diff_theta, abs(delta_theta)-0.05, rot_speed)

        if diff_theta > abs(delta_theta)-0.1:
            client.set_robot_velocity(0, 0)
            print("break")
            break

        client.set_robot_velocity(0, rot_speed)

    return True

def move_forward( dist ):
    client.set_manual_control_enabled(True)
    pose = client.get_robot_pose()
    init_x = pose.x
    init_y = pose.y 


    while 1:
        pose = client.get_robot_pose()
        cur_x = pose.x
        cur_y = pose.y 

        diff = np.linalg.norm( [init_x-cur_x, init_y-cur_y  ] )

        speed = np.clip( abs(dist-diff), 0.1, 0.3 )
        if dist<0:
            speed *= -1

        client.set_robot_velocity(speed, 0 )

        if diff > abs(dist)-0.03:
            client.set_robot_velocity(0, 0)
            break

    return True

def normalize_angle( radian ):
    while 1:
        if radian > math.pi:
            radian -= 2 * math.pi
        elif radian < -math.pi:
            radian += 2 * math.pi
        else:
            break

    return radian

def fine_tune_pose( x, y, theta ):
    cur_pose = client.get_robot_pose()
    delta_theta = math.atan2( y-cur_pose.y, x-cur_pose.x ) - cur_pose.theta
    delta_theta = normalize_angle( delta_theta )

    print("進行方向回転：", delta_theta)
    rotate_in_place( delta_theta )

    dist = np.linalg.norm( [x-cur_pose.x, y-cur_pose.y] )
    print("直進：", dist)
    move_forward( dist )

    delta_theta = theta - client.get_robot_pose().theta
    delta_theta = normalize_angle( delta_theta )
    rotate_in_place( delta_theta )

    return True


def service_server( req ):
    data = yaml.safe_load( req.data )
    print(f"revieved data: {data}")

    com = data["command"]
    args = data["args"]

    functions = {
        # コマンド名: (実行する関数，引数，戻り値の変換関す)
        "speak": ( client.speak, (str,), str ),
        "return_home": ( client.return_home, (), str ),
        "get_locations": ( client.get_locations, (), str ),
        "get_robot_pose": ( client.get_robot_pose, (), lambda pose: (pose.x, pose.y, pose.theta) ),
        "move_shelf": (client.move_shelf, (str, str), str ),
        "move_to_location": (client.move_to_location, (str,), str),
        "move_to_pose": (client.move_to_pose, (float,float,float), str), 
        "return_shelf": (client.return_shelf, (), str),
        "rotate_in_place": (rotate_in_place, (float,), str),
        "move_forward": (move_forward, (float,), str),
        "fine_tune_pose": (fine_tune_pose, (float, float, float), str),
        "dock_shelf": (client.dock_shelf, (), str),
    }

    result = False
    data = None
    msg = ""
    if com in functions:
        func = functions[com][0]                # 実行する関数
        arg_types = functions[com][1]           # 引数の型
        conv_return_value = functions[com][2]   # 戻り値の変換関数

        # 引数の型をチェック
        args, msg = check_types( com, args, arg_types )
        if args is not None:
            if len(args)!=0:
                data = func( *args )
            else:
                data = func()
            
            # 戻り値を変換
            data = conv_return_value( data )

            result = True
    else:
        result = False
        msg = f"{com} is invalid command"

    print(msg)
    return StringTriggerResponse( yaml.safe_dump( {"result":result, "data":data, "msg":msg} ) )

def manual_control( twist ):
    client.set_manual_control_enabled(True)
    client.set_robot_velocity(twist.linear.x, twist.angular.z)

def main():
    action_server = actionlib.SimpleActionServer("move_base", MoveBaseAction, execute_cb=move_base_action_server)
    action_server.start()

    rospy.Subscriber("/move_base_simple/goal", PoseStamped, move_base_action_server)
    rospy.Subscriber("/kachaka/speak", String, speak)
    rospy.Subscriber("/cmd_vel", Twist, manual_control)

    th_sending = threading.Thread(target=sending_thread, daemon=True)
    th_sending.start()

    # mapをpublish
    publish_map()

    # サービス実行
    rospy.Service("kachaka_func", StringTrigger, service_server)

    rospy.spin()

    

if __name__=="__main__":
    ip = sys.argv[1]

    rospy.init_node("kachaka")
    client = kachaka_api.KachakaApiClient( ip + ":26400")
    client.set_auto_homing_enabled(False)

    br = tf.TransformBroadcaster()

    main()

