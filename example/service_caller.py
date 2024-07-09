import rospy
from ros_msgs.srv import StringTrigger
import yaml

def kachaka_func( com, args=[] ):
    data = yaml.safe_dump( { "command":com, "args":args } )
    ret = kachaka_service( data )
    ret = yaml.safe_load( ret.result )
    return ret["result"], ret["data"], ret["msg"]

def main():
    functions = [ "speak", "return_home", "get_locations", "get_robot_pose", "move_shelf", "move_to_location", "move_to_pose", "return_shelf", "rotate_in_place", "move_forward" ]
    functions = sorted(functions)

    while 1:
        print("-----------------------------")
        print("Select a command: ")
        for i, f in enumerate(functions):
            print( f"{i}: {f}" )
        i = int(input(" -> "))

        print( "Input arguments:" )
        args = input(" -> ")

        if args!="":
            args = args.split()
        else:
            args = []

        # サービス実行
        res, data, msg = kachaka_func( functions[i], args )

        print("**********")
        print(f"result:{res}")
        print(f"msg:{msg}")
        print(f"data:{data}")
        print("**********")

if __name__=="__main__":
    rospy.init_node( "kachaka_service_caller" )  
    kachaka_service = rospy.ServiceProxy("kachaka_func", StringTrigger)
    main()

