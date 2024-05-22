#!/usr/bin/env python3

import argparse
import datetime
import sd_data_adapter.models as models
from sd_data_adapter.client import DAClient
from sd_data_adapter.api import upload, get_by_id, update

def send_command_message(id):
    model = models.autonomous_mobile_robot.CommandMessage(id)
    model.command="AutonomousNavigation"
    model.commandTime=str(datetime.datetime.now())
    model.waypoints=[{
                        "geographicPoint": {
                          "latitude": 42.16381303262392,
                          "longitude": 3.0930211519197712,
                          "altitude": 0.0
                        },
                        "orientation3D": {
                          "roll": 0.0,
                          "pitch": 0.0,
                          "yaw": -1.571
                        }
                      },
                      {
                        "geographicPoint": {
                          "latitude": 42.16285052640879,
                          "longitude": 3.0929409802538714,
                          "altitude": 0.0
                        },
                        "orientation3D": {
                          "roll": 0.0,
                          "pitch": 0.0,
                          "yaw": -1.571
                        }
                      }]
    upload(model)
    
    print(f"Uploaded CommandMessage to Context Broker with id {id}")

def send_state_message(id):
    model = models.autonomous_mobile_robot.StateMessage(id)
    model.destination = {"geographicPoint": {"latitude": 42.16285052640879,"longitude": 3.0929409802538714,"altitude": 0.0},"orientation3D": {"roll": 0.0,"pitch": 0.0,"yaw": -1.571}}
    model.battery = 50.0
    model.commandTime = str(datetime.datetime.now())
    model.mode = "Spraying"
    model.pose = {"geographicPoint": {"latitude": 42.16395021706663,"longitude": 3.0929278979099024,"altitude": 0.0},"orientation3D": {"roll": 0.0,"pitch": 0.0,"yaw": -1.571}}
    upload(model)

    print(f"Uploaded StateMessage to Context Broker with id {id}")

def main():
    parser = argparse.ArgumentParser(description="Upload a entity with a concrete ID to Context Broker")
    parser.add_argument("entity", choices=["CommandMessage", "StateMessage"], help="Type of entity to upload")
    parser.add_argument("id", help="ID of the entity to publish")

    args = parser.parse_args()

    if args.entity == "CommandMessage":
        send_command_message(args.id)
    elif args.entity == "StateMessage":
        send_state_message(args.id)
    else:
        print("Unknown entity type")

if __name__ == "__main__":
    main()