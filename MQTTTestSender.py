from MQTTGenericClient import MQTTGenericClient
import time
import random
from Setup import BFMC_MQTT_CONTROL_TOPIC

mqttControlMessage = MQTTGenericClient(f"client1{random.randint(1, 1000)}",qos=1)
mqttControlMessage.start_client()

counter = 0

while True:
    time.sleep(10)
    mqttControlMessage.publish_message(f"Hello World {counter}", BFMC_MQTT_CONTROL_TOPIC)
    counter += 1