import json
import os
import time


class GenericJsonReader:
    def __init__(self, json_file_name):
        self.json_file_satus = "Free"
        self.name = json_file_name
        self.json_current_message = None
        self.json_current_message_id = None
        self.json_current_message_topic = None

    def check_if_file_exists(self):
        if self.name not in os.listdir():
            f = open(self.name, 'w')
            f.write("{}")
            f.close()

    def write_message_to_json(self, message, topic_name=None):
        assert message is not None
        assert topic_name is not None

        while self.json_file_satus == "Busy":
            # print("write_message_to_json: JSON file is busy, waiting 0.5s")
            time.sleep(0.5)
        self.json_file_satus = "Busy"
        print("write_message_to_json: JSON made busy")

        print("writing to json")
        f = None
        json_content = None
        last_key = None

        self.check_if_file_exists()

        f = open(self.name, 'r+')
        json_content = json.load(f)

        if topic_name not in json_content:
            json_content[topic_name] = {}
            last_key = 0
        elif len(json_content[topic_name]) > 0:
            last_key = list(json_content[topic_name].keys())[-1]
        else:
            last_key = 0

        json_message_id = int(last_key) + 1

        # Add a new entry to data with message and status
        if isinstance(message, bytes):
            message = message.decode('utf-8')
        json_content[topic_name][json_message_id] = {"message": message, "status": "new"}
        json_message_id += 1

        f.seek(0)  # Move the file pointer to the beginning of the file
        f.write(json.dumps(json_content))
        f.truncate()  # Truncate the file to the current position (i.e., the end of the JSON data)
        f.close()
        self.json_file_satus = "Free"
        print("message written to json, json file is free again")

    def get_next_message(self, topic_name, system_start=False):
        self.mark_current_message_as_finished(topic_name)

        while self.json_file_satus == "Busy":
            # print("get_next_message: JSON file is busy, waiting 0.5s")
            time.sleep(0.5)
        self.json_file_satus = "Busy"
        print("get_next_message: JSON made busy")


        next_message = None
        self.check_if_file_exists()

        with open(self.name, 'r') as json_file:
            json_content = json.load(json_file)
            if topic_name not in json_content:
                print(f"{topic_name} not found in json file")
                self.json_file_satus = "Free"
                return False

            all_messages = json_content[topic_name]

            if len(all_messages) > 0:
                if system_start or all_messages[list(all_messages.keys())[0]]["status"] != "in_progress":
                    print("extracting next message from json")
                    self.json_current_message_id = list(all_messages.keys())[0]
                    self.json_current_message = all_messages[self.json_current_message_id]["message"]
                    self.json_current_message_topic = topic_name
                    json_current_status = all_messages[self.json_current_message_id]["status"]
                    next_message = self.json_current_message

                    if json_current_status == "new":
                        all_messages[self.json_current_message_id]["status"] = "in_progress"
                    # elif json_current_status == "finished":
                    #     # from all_messages, remove the first element
                    #     all_messages.pop(self.json_current_message_id)
                    #
                    #     if len(all_messages) > 0:
                    #         self.json_current_message_id = list(all_messages.keys())[0]
                    #         self.json_current_message = all_messages[self.json_current_message_id]["message"]
                    #         all_messages[self.json_current_message_id]["status"] = "in_progress"

                    # write back to json file
                    json_content[topic_name] = all_messages
                    with open(self.name, 'r+') as f:
                        json.dump(json_content, f)
                else:
                    print("Current message is in progress")

        self.json_file_satus = "Free"
        return next_message

    def mark_current_message_as_finished(self, topic_name):
        while self.json_file_satus == "Busy":
            # print("mark_current_message_as_finished: JSON file is busy, waiting 0.5s")
            time.sleep(0.5)
        self.json_file_satus = "Busy"
        print("mark_current_message_as_finished: JSON made busy")

        if self.json_current_message_id is None:
            print("No current message to mark as finished")
            self.json_file_satus = "Free"
            return False

        self.check_if_file_exists()

        with open(self.name , 'r+') as json_file:
            json_content = json.load(json_file)
            if topic_name not in json_content:
                print(f"{topic_name} not found in json file")
                self.json_file_satus = "Free"
                return False

            all_messages = json_content[self.json_current_message_topic]

            if self.json_current_message_id in all_messages.keys():
                #delete element with self.json_current_message_id
                all_messages.pop(self.json_current_message_id)
                self.json_current_message = None
                self.json_current_message_id = None
                self.json_current_message_topic = None

                json_file.seek(0)  # Move the file pointer to the beginning of the file
                json_file.write(json.dumps(json_content))
                json_file.truncate()
            print("Current message successfully finished")

        self.json_file_satus = "Free"

    def get_current_message(self):
        return self.json_current_message

    def get_current_message_id(self):
        return self.json_current_message_id


if __name__ == "__main__":
    json_file_name = "MQTTVehicleControlMessages.json"
    topic_name = "planned_activities"
    json_reader = GenericJsonReader(json_file_name)

    print(json_reader.get_next_message("house/light"))
    json_reader.mark_current_message_as_finished("house/light")
