#https://smartdatamodels.org/dataModel.AutonomousMobileRobot/CommandMessage
#https://github.com/smart-data-models/dataModel.AutonomousMobileRobot/tree/297af1c06a790884c361b1f479f40f424e64678d/CommandMessage

std_msgs/Header                                        header

string                                                 command
string                                                 command_time
string                                                 type
retrofitted_tractor_data_adapters/GeographicPose[]     waypoints  

