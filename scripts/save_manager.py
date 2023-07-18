#!/usr/bin/env python3

import pickle
import json
import csv
import os
from datetime import datetime

class SaveManager():
    """Allows for saving and loading joints in multiple different formats.
    
    Formats : pickle, json, csv. Saves to and loads from working directory.
    Saving and loading to csv only supports the original format with value 
    for each key being a tuple containing one 3D array and one 4D array."""

    def save_any(self, joints:dict, file_name:str) -> None:
        extension = file_name.split(".")[-1]
        if extension == "pkl":
            self.save_pickle(joints, file_name)
        if extension == "json":
            self.save_json(joints, file_name)
        if extension == "csv":
            self.save_csv(joints, file_name)

    def load_any(self, file_name:str) -> dict:
        extension = file_name.split(".")[-1]
        if extension == "pkl":
            return self.load_pickle(file_name)
        if extension == "json":
            return self.load_json(file_name)
        if extension == "csv":
            return self.load_csv(file_name)
        self.file_not_found(file_name)
        return {}

    def check_extension(self, file_name:str, extension:str) -> str:
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

    def save_pickle(self, joints:dict, file_name:str="joints.pkl") -> None:
        file_name = self.check_extension(file_name, "pkl")
        with open(file_name, "wb") as file:
            pickle.dump(joints, file)

    def load_pickle(self, file_name:str="joints.pkl") -> dict:
        try:
            file_name = self.check_extension(file_name, "pkl")
            joints:dict
            with open(file_name, "rb") as file:
                joints = pickle.load(file)
            return joints
        except FileNotFoundError:
            self.file_not_found(file_name)
            return {}
    
    def save_json(self, joints:dict, file_name:str="joints.json") -> None:
        file_name = self.check_extension(file_name, "json")
        with open(file_name, "w") as file:
            json.dump(joints, file)

    def load_json(self, file_name:str="joints.json") -> dict:
        try:
            file_name = self.check_extension(file_name, "json")
            joints:dict
            with open(file_name, "r") as file:
                joints = json.load(file)
            return joints
        except FileNotFoundError:
            self.file_not_found(file_name)
            return {}
    
    def save_csv(self, joints:dict, file_name:str="joints.csv") -> None:
        file_name = self.check_extension(file_name, "csv")
        with open(file_name, "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(['Joint', 'Translation', 'Rotation'])
            # Write the values as data rows
            for key, value in joints.items():
                translation = json.dumps(value[0])
                rotation = json.dumps(value[1])
                writer.writerow([key, translation, rotation])

    def load_csv(self, file_name:str="joints.csv") -> dict:
        try:
            file_name = self.check_extension(file_name, "csv")
            joints = {}
            with open(file_name, "r", newline="") as file:
                reader = csv.DictReader(file)
                for row in reader:
                    joint = row['Joint']
                    translation = json.loads(row['Translation'])
                    rotation = json.loads(row['Rotation'])
                    joints[joint] = [translation, rotation]
            return joints
        except FileNotFoundError:
                self.file_not_found(file_name)
                return {}


if __name__ == "__main__":
    SM = SaveManager()

    joints = {"test_joint" : [[0.1, 0.2, 0.3], [0.1, 0.2, 0.3, 0.4]]}

    SM.save_pickle(joints)
    SM.save_json(joints)
    SM.save_csv(joints)

    assert(SM.load_pickle("joints") == joints)
    assert(SM.load_json("joints") == joints)
    assert(SM.load_csv("joints") == joints)