import paho.mqtt.client as paho
from paho import mqtt
import threading
import queue

class MQTTGenericClient:
    def __init__(self, client_id, qos=1, json_file_object=None):
        self.message_queue = queue.Queue()
        self.connected = False
        self.json_message_id = 0
        self.client = None
        self.json_file_object = json_file_object
        self.client_thread = None
        self.json_write_thread = None
        self.qos = qos
        self.last_message = None
        self.client_id = client_id

    # setting callbacks for different events to see if it works, print the message etc.
    def on_connect(self, client, userdata, flags, rc, properties=None):
        self.connected = True
        print("CONNACK received with code %s." % rc)
        print("Connected to MQTT broker successfully")

    # Define the callback function for when the MQTT client disconnects
    def on_disconnect(self, client, userdata, rc):
        print("Disconnected from MQTT broker with code %s." % rc)
        self.connected = False

    # with this callback you can see if your publish was successful
    def on_publish(self, client, userdata, mid, properties=None):
        print("mid: " + str(mid))

    # print which topic was subscribed to
    def on_subscribe(self, client, userdata, mid, granted_qos, properties=None):
        print("Subscribed: " + str(mid) + " " + str(granted_qos))

    # print message, useful for checking if it was successful
    def on_message(self, client, userdata, msg):
        print("got message: " + msg.topic + " " + str(msg.qos) + " " + str(msg.payload))
        self.message_queue.put([msg.payload, msg.topic])
        print("message queue size: " + str(self.message_queue.qsize()))
        self.start_json_write_thread()

    def mqtt_loop(self):
        self.client.loop_forever()

    def start_client(self):
        # using MQTT version 5 here, for 3.1.1: MQTTv311, 3.1: MQTTv31
        # userdata is user defined data of any type, updated by user_data_set()
        # client_id is the given name of the client
        self.client = paho.Client(client_id=self.client_id, userdata=None, protocol=paho.MQTTv5)
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
        self.start_client_thread()

    def subscribe(self, topic_name):
        # subscribe to all topics by using the wildcard "#"
        self.client.subscribe(topic_name, qos=self.qos)
        # self.start_json_write_thread()

    def publish_message(self, message=None, topic_name=None):
        assert message is not None
        assert topic_name is not None
        if self.connected:
            self.client.publish(topic_name, message, qos=self.qos)

    def get_messages(self):
        if self.connected:
            try:
                # Get the next frame from the queue with a timeout of 1 second
                self.last_message, topic = self.message_queue.get(timeout=1)#.decode('utf-8')
                print(f"Got message from queue '{self.last_message}' from topic '{topic}'")

                if self.json_file_object is not None:
                    self.json_file_object.write_message_to_json(self.last_message, topic)

            except queue.Empty:
                pass

    def start_client_thread(self):
        self.client_thread = threading.Thread(target=self.mqtt_loop)
        self.client_thread.start()

    def stop_client_thread(self):
        self.client_thread.stop()

    def start_json_write_thread(self):
        self.json_write_thread = threading.Thread(target=self.get_messages)
        self.json_write_thread.start()

