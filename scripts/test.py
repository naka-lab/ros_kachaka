import sys
import kachaka_api

client = kachaka_api.KachakaApiClient("192.168.1.75:26400")

print(client.get_shelves())

client.move_shelf("S02", "L01")
client.return_shelf()
client.return_home()