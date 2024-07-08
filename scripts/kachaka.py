import rospy
import tf
import time
import sys
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
from geometry_msgs.msg import PoseStamped

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

def main():
    # map保存してpublish
    save_map( "map" )
    subprocess.run("rosrun map_server map_server map.yaml &", shell=True)


    #client.move_shelf("S02", "L01")
    #client.return_shelf()

    #client.move_to_location("L01")
    #client.move_to_pose( 0.3, 0, 0)

    #client.return_home()

    rospy.spin()

    

if __name__=="__main__":
    rospy.init_node("kachaka")
    ip = sys.argv[1]
    client = kachaka_api.KachakaApiClient( ip + ":26400")
    print(dir(client))
    client.set_auto_homing_enabled(False)

    br = tf.TransformBroadcaster()

    action_server = actionlib.SimpleActionServer("move_base", MoveBaseAction, execute_cb=move_base_action_server)
    action_server.start()

    rospy.Subscriber("/move_base_simple/goal", PoseStamped, move_base_action_server)
    rospy.Subscriber("/kachaka/speak", String, speak)

    th_sending = threading.Thread(target=sending_thread, daemon=True)
    th_sending.start()
    main()

