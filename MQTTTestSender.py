from MQTTGenericClient import MQTTGenericClient
import time

mqttControlMessage = MQTTGenericClient("client1",qos=1)
mqttControlMessage.start_client()

counter = 0

while True:
    time.sleep(30)
    mqttControlMessage.publish_message(f"Hello World {counter}", "house/light")
    counter += 1