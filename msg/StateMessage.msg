#https://smartdatamodels.org/dataModel.AutonomousMobileRobot/StateMessage
#https://github.com/smart-data-models/dataModel.AutonomousMobileRobot/tree/297af1c06a790884c361b1f479f40f424e64678d/StateMessage

std_msgs/Header                                        header

float32[]                                              accuracy
float32                                                battery
string                                                 command_time
retrofitted_tractor_data_adapters/GeographicPose       destination
uint8[]                                                errors
string                                                 mode
retrofitted_tractor_data_adapters/GeographicPose       pose
string                                                 type

