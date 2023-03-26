import json

class ActivityJsonReader:
    def __init__(self, json_file_name, topic_name):
        self.json_file_name = json_file_name
        self.json_current_message = None
        self.json_current_message_id = None
        self.topic_name = topic_name

    def get_next_message(self):
        with open(self.json_file_name) as json_file:
            try:
                json_content = json.load(json_file)
                assert self.topic_name in json_content, f"Topic {self.topic_name} not found in JSON file"
            except json.decoder.JSONDecodeError:
                print("JSON file is empty")
                return

            all_messages = json_content[self.topic_name]

            self.json_current_message_id = list(all_messages.keys())[0]
            self.json_current_message = all_messages[self.json_current_message_id]["message"]
            json_current_status = all_messages[self.json_current_message_id]["status"]

            if json_current_status == "new":
                all_messages[self.json_current_message_id]["status"] = "in_progress"
            elif json_current_status == "in_progress":
                #activity not done, need to wait
                return None
            elif json_current_status == "finished":
                #from all_messages, remove the first element
                all_messages.pop(self.json_current_message_id)
                self.json_current_message_id = list(all_messages.keys())[0]
                self.json_current_message = all_messages[self.json_current_message_id]["message"]
                all_messages[self.json_current_message_id]["status"] = "in_progress"

            #write back to json file
            json_content[self.topic_name] = all_messages
            with open(self.json_file_name, 'w') as f:
                json.dump(json_content, f)

        return self.json_current_message

    def mark_current_message_as_finished(self):
        if self.json_current_message_id is None:
            print("No current message to mark as finished")
            return False

        with open(self.json_file_name) as json_file:
            try:
                json_content = json.load(json_file)
                assert self.topic_name in json_content, f"Topic {self.topic_name} not found in JSON file"
            except json.decoder.JSONDecodeError:
                print("JSON file is empty")
                return

            all_messages = json_content[self.topic_name]
            all_messages[self.json_current_message_id]["status"] = "finished"

            #write back to json file
            json_content[self.topic_name] = all_messages
            with open(self.json_file_name, 'w') as f:
                json.dump(json_content, f)
            print("Current activity successfully marked as finished")

    def get_current_message(self):
        return self.json_current_message

    def get_current_message_id(self):
        return self.json_current_message_id


if __name__ == "__main__":
    json_file_name = "planned_activities.json"
    topic_name = "planned_activities"
    json_reader = ActivityJsonReader(json_file_name, topic_name)
    json_reader.mark_current_message_as_finished()
    json_reader.get_next_message()
    json_reader.mark_current_message_as_finished()
    json_reader.get_next_message()
    json_reader.mark_current_message_as_finished()
    json_reader.get_next_message()
    json_reader.mark_current_message_as_finished()
    print(json_reader.get_current_message())
    print(json_reader.get_current_message_id())