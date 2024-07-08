import sys
import kachaka_api

client = kachaka_api.KachakaApiClient("192.168.1.75:26400")

print(client.get_locations())

pose = client.get_ros_odometry().pose.pose
print(pose.position.x)
print(pose.orientation.w)


client.move_shelf("S02", "L01")
client.return_shelf()
client.return_home()