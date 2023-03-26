import paho.mqtt.client as paho
from paho import mqtt
import threading
import queue
import json


class MQTTGenericClient:
    def __init__(self, qos=1, json_file_name=None):
        self.topic_name = None
        self.message_queue = queue.Queue()
        self.connected = False
        self.json_message_id = 0
        self.client = None
        self.json_file_name = json_file_name
        self.message_receiving_thread = None
        self.qos = qos
        self.last_message = None

    # setting callbacks for different events to see if it works, print the message etc.
    def on_connect(self, client, userdata, flags, rc, properties=None):
        self.connected = True
        print("CONNACK received with code %s." % rc)
        print("Connected to MQTT broker successfully")

    # Define the callback function for when the MQTT client disconnects
    def on_disconnect(self, client, userdata, rc):
        self.connected = False

    # with this callback you can see if your publish was successful
    def on_publish(self, client, userdata, mid, properties=None):
        print("mid: " + str(mid))

    # print which topic was subscribed to
    def on_subscribe(self, client, userdata, mid, granted_qos, properties=None):
        print("Subscribed: " + str(mid) + " " + str(granted_qos))

    # print message, useful for checking if it was successful
    def on_message(self, client, userdata, msg):
        print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))
        self.message_queue.put([msg.payload, msg.topic])

    def start_client(self):
        # using MQTT version 5 here, for 3.1.1: MQTTv311, 3.1: MQTTv31
        # userdata is user defined data of any type, updated by user_data_set()
        # client_id is the given name of the client
        self.client = paho.Client(client_id="client2", userdata=None, protocol=paho.MQTTv5)
        self.client.on_connect = self.on_connect

        # enable TLS for secure connection
        self.client.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
        # set username and password
        self.client.username_pw_set("bfmc_iseliikur", "iseliikur")
        # connect to HiveMQ Cloud on port 8883 (default for MQTT)
        self.client.connect("66def72102a14a59848581282107fb3f.s2.eu.hivemq.cloud", 8883)

        # setting callbacks, use separate functions like above for better visibility
        self.client.on_subscribe = self.on_subscribe
        self.client.on_message = self.on_message
        self.client.on_publish = self.on_publish

    def subscribe(self, topic_name):
        self.topic_name = topic_name
        # subscribe to all topics of encyclopedia by using the wildcard "#"
        self.client.subscribe(self.topic_name, qos=self.qos)

    def publish_message(self, message):
        # while self.connected:
        #     self.client.publish(self.topic_name, message, qos=self.qos)
        pass

    def get_messages(self):
        while self.connected:
            try:
                # Get the next frame from the queue with a timeout of 1 second
                self.last_message, topic = self.message_queue.get(timeout=1).decode('utf-8')
                print(f"Got message {self.last_message} from topic {topic}")

                if self.json_file_name is not None:
                    self.write_to_json(self.last_message)

            except queue.Empty:
                pass

    def write_to_json(self, message):
        # Open the JSON file in read mode
        with open(self.json_file_name, 'r') as f:
            # Load the contents of the file into a Python object
            try:
                json_content = json.load(f)
            except json.decoder.JSONDecodeError:
                print("JSON file is empty")
                json_content = {}

        if self.topic_name not in json_content:
            json_content[self.topic_name] = {}

        last_key = list(json_content[self.topic_name].keys())[-1]
        self.json_message_id = int(last_key) + 1

        # Add a new entry to data with message and status
        json_content[self.topic_name][self.json_message_id] = {"message": message, "status": "new"}
        self.json_message_id += 1

        # Open the same file in write mode
        with open(self.json_file_name, 'w') as f:
            # Write the modified Python object back to the file
            json.dump(json_content, f)

    def start_subscriber_thread(self):
        self.message_receiving_thread = threading.Thread(target=self.get_messages)
        self.message_receiving_thread.start()

    def stop_subscriber_thread(self):
        self.message_receiving_thread.stop()
