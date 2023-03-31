from MQTTGenericClient import MQTTGenericClient
from GenericJsonReader import GenericJsonReader

json_file = GenericJsonReader("MQTTVehicleControlMessages.json")

mqttControlMessage = MQTTGenericClient("client2", 1, json_file)
mqttControlMessage.start_client()
mqttControlMessage.subscribe("house/light")

# json_file.mark_current_message_as_finished("house/light")
# print(json_file.get_next_message("house/light"))