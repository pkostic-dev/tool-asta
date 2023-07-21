#!/usr/bin/env python3

import pickle
import json
import csv
import os
from datetime import datetime


class SaveManager():
    """Allows for saving and loading joints in multiple different formats.

    Available formats : pickle, json, csv. Saves to and loads from
    working directory. Saving and loading to csv only supports the
    original format with value for each key being a tuple containing one
    3D array and one 4D array."""

    def save_any(self, joints: dict, file_name: str) -> None:
        extension = file_name.split(".")[-1]
        if extension == "pkl":
            self.save_dict_to_pickle(joints, file_name)
        if extension == "json":
            self.save_dict_to_json(joints, file_name)
        if extension == "csv":
            self.save_dict_to_csv(joints, file_name)

    def load_any(self, file_name: str) -> dict:
        extension = file_name.split(".")[-1]
        if extension == "pkl":
            return self.load_pickle_to_dict(file_name)
        if extension == "json":
            return self.load_json_to_dict(file_name)
        if extension == "csv":
            return self.load_csv_to_dict(file_name)
        self.file_not_found(file_name)
        return {}

    def check_extension(self, file_name: str, extension: str) -> str:
        if not file_name:
            now = datetime.now()
            now_str = now.strftime("-%Y-%m-%d-%H-%M-%S")
            return "joints" + now_str + "." + extension
        file_extension = file_name.split(".")[-1]
        if file_extension != extension:
            return file_name + "." + extension
        return file_name

    def file_not_found(self, file_name) -> None:
        print("File not found : " + file_name)
        print("Files in working directory : ")
        files_list = list(filter(os.path.isfile, os.listdir(".")))
        for f in files_list:
            print("\t", f)

    def save_dict_to_pickle(self, data: dict, file_name: str = "data.pkl") -> None:
        file_name = self.check_extension(file_name, "pkl")
        with open(file_name, "wb") as file:
            pickle.dump(data, file)

    def load_pickle_to_dict(self, file_name: str = "data.pkl") -> dict:
        try:
            file_name = self.check_extension(file_name, "pkl")
            data: dict
            with open(file_name, "rb") as file:
                data = pickle.load(file)
            return data
        except FileNotFoundError:
            self.file_not_found(file_name)
            return {}

    def save_dict_to_json(self, data: dict, file_name: str = "data.json") -> None:
        # NOTE : tuple is transformed into list
        file_name = self.check_extension(file_name, "json")
        with open(file_name, "w") as file:
            json.dump(data, file)

    def load_json_to_dict(self, file_name: str = "data.json") -> dict:
        try:
            file_name = self.check_extension(file_name, "json")
            data: dict
            with open(file_name, "r") as file:
                data = json.load(file)
            return data
        except FileNotFoundError:
            self.file_not_found(file_name)
            return {}

    def save_dict_to_csv(self, data: dict, file_name: str = "data.csv") -> None:
        file_name = self.check_extension(file_name, "csv")
        with open(file_name, "w", newline="") as file:
            writer = csv.writer(file)
            first_element_type = next(iter(data.values()))
            if isinstance(first_element_type, list):
                writer.writerow(['Joint', 'Translation', 'Rotation'])
                # Write the values as data rows
                for key, value in data.items():
                    translation = json.dumps(value[0])
                    rotation = json.dumps(value[1])
                    writer.writerow([key, translation, rotation])
            elif isinstance(first_element_type, dict):
                joint_keys = [key for key in next(
                    iter(data.values()))['joints']]
                writer.writerow(['JointKeys', 'JointA', 'JointVertex',
                                 'JointB', 'Degrees'])
                for item in data.values():
                    joint_values = [json.dumps(item['joints'][key])
                                    for key in joint_keys]
                    degrees = item['degrees']
                    writer.writerow([joint_keys] + joint_values + [degrees])

    def load_csv_to_dict(self, file_name:str = "data.csv") -> dict:
        try:
            file_name = self.check_extension(file_name, "csv")
            data = {}
            with open(file_name, "r", newline="") as file:
                reader = csv.DictReader(file)
                fieldnames = reader.fieldnames
                columns = 0
                # NOTE : fieldnames contains Sequence | None so need to check
                if fieldnames:
                    columns = len(fieldnames)
                if columns == 3:
                    for row in reader:
                        joint = row['Joint']
                        translation = json.loads(row['Translation'])
                        rotation = json.loads(row['Rotation'])
                        data[joint] = [translation, rotation]
                if columns == 5:
                    for row in reader:
                        # NOTE: json.loads doesn't like quotes
                        joint_keys = row['JointKeys'].replace("'", "~")
                        joint_keys = row['JointKeys'].replace('"', "'")
                        joint_keys = row['JointKeys'].replace("'", '"')
                        joint_keys = json.loads(joint_keys)
                        vertex_key = joint_keys[1]
                        data[vertex_key] = {
                            "joint_keys" : joint_keys,
                            "joints" : {
                                joint_keys[0] : json.loads(row['JointA']),
                                vertex_key : json.loads(row['JointVertex']),
                                joint_keys[2] : json.loads(row['JointB'])
                            },
                            "degrees" : json.loads(row['Degrees'])
                        }
            return data
        except FileNotFoundError:
            self.file_not_found(file_name)
            return {}


if __name__ == "__main__":
    SM = SaveManager()

    print("Test module ? (y/n) ", end="")
    test = input()
    if test == "y" or test == "Y":
        joints = {"test_joint": [[0.1, 0.2, 0.3], [0.1, 0.2, 0.3, 0.4]]}
        angles = {
            "vertex" : {
                "joint_keys" : ["joint_a", "vertex", "joint_b"],
                "joints" : {
                    "joint_a" : [[0.1, 0.2, 0.3], [0.1, 0.2, 0.3, 0.4]],
                    "vertex" : [[0.1, 0.2, 0.3], [0.1, 0.2, 0.3, 0.4]],
                    "joint_b" : [[0.1, 0.2, 0.3], [0.1, 0.2, 0.3, 0.4]],
                },
                'degrees' : 90.0
            }
        }

        SM.save_dict_to_pickle(joints, "joints")
        SM.save_dict_to_json(joints, "joints")
        SM.save_dict_to_csv(joints, "joints")

        SM.save_dict_to_pickle(angles, "angles")
        SM.save_dict_to_json(angles, "angles")
        SM.save_dict_to_csv(angles, "angles")

        assert (SM.load_pickle_to_dict("joints") == joints)
        assert (SM.load_json_to_dict("joints") == joints)
        assert (SM.load_csv_to_dict("joints") == joints)

        assert (SM.load_pickle_to_dict("angles") == angles)
        assert (SM.load_json_to_dict("angles") == angles)
        assert (SM.load_csv_to_dict("angles") == angles)

        print("All tests passed.")
