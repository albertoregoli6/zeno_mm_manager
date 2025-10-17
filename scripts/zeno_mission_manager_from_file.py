#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import yaml 
import sys
import os 

from zeno_mission_manager.msg import WaypointList, Waypoint
from std_msgs.msg import String, Empty


# import the ergo python macros 
mm_path   = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir))
ergo_path = os.path.abspath(os.path.join(mm_path, os.pardir))
sys.path.append(ergo_path+'/ergo_lib/py_lib/')
HOME = os.path.expanduser('~')

class Zeno_Mission_Manager_From_File(object):
    #---------------------------------------------------------------------------------------
    #////////////////////////////        INIT         //////////////////////////////////////
    #---------------------------------------------------------------------------------------
    def __init__(self):
        self.yaml_mission_folder    	= rospy.get_param("missions_repository");
        # self.proprietary_mission_folder	= rospy.get_param("proprietary_mission_folder");
        self.pub_mission 				= rospy.Publisher('mm/upload_mission', WaypointList, queue_size=1, latch=True)
        self.mission_loaded_pub         = rospy.Publisher('mm/mission_loaded', String, queue_size=1, latch=True)
        self.sub_load_mission_file 		= rospy.Subscriber("mm/load_mission_file", String, self.load_mission_file_callback, tcp_nodelay=True, queue_size=1)
        self.sub_get_mission_list  		= rospy.Subscriber("mm/get_mission_list", Empty, self.get_mission_list_callback, tcp_nodelay=True, queue_size=1)

    #---------------------------------------------------------------------------------------
    #/////////////////////////        SUPPORT FUNCTIONs         ////////////////////////////
    #---------------------------------------------------------------------------------------
    # Read the selected mission file, parse yaml and publish the first task
    def load_mission_file(self, filename):
    	# open the yaml file from which load the mission
    	file_path 	= HOME + self.yaml_mission_folder + '/'+filename+'.yaml'
        if os.path.exists(file_path):
        	try:
        		file_struct = open(file_path,'r')
        		print(file_struct)
        	except:
        		rospy.logerr("[ZENO MISSION MANAGER FROM FILE] Error while opening the yaml file. File path: [%s]", file_path)
        		return False
        	# load the mission from the opened yaml file
        	try:
        		mission_struct = yaml.safe_load(file_struct)
        	except:
        		rospy.logerr("[ZENO MISSION MANAGER FROM FILE] Error while loading the file content.")
        		return False
        	file_struct.close() # close the file
        	# get the tasks list from the loaded mission file
        	try:
        		self.parse_mission(mission_struct)
        	except:
        		rospy.logerr("[ZENO MISSION MANAGER FROM FILE] Error while parsing the file content.")
        		return False

        	return True
        else:
            rospy.logerr("[ZENO MISSION MANAGER FROM FILE] Mission file does not exists.")
            return False

    # ---------------------------------------------------------------------------------------
    # ---------------------------------------------------------------------------------------
    # parse the mission structure into a list of tasks
    def parse_mission(self, mission_struct):
    	self.loaded_mission = WaypointList()
    	print(mission_struct["tasks"])
    	for task in mission_struct["tasks"]:
    		waypoint_msg = Waypoint()
    		waypoint_msg.position.latitude 		= task["coordinates"]["latitude"]
    		waypoint_msg.position.longitude 	= task["coordinates"]["longitude"]
    		waypoint_msg.position.depth 		= task["depth"]
    		waypoint_msg.speed 					= task["speed"]
    		waypoint_msg.control_mode 			= "depth"
    		self.loaded_mission.waypoint_list 	+= [waypoint_msg]

    #---------------------------------------------------------------------------------------
    #//////////////////////////        CALLBACK         ////////////////////////////////////
    #---------------------------------------------------------------------------------------
    def get_mission_list_callback(self, msg):
    	mission_list = []
    	for file in os.listdir(HOME + self.yaml_mission_folder):
    		if file.endswith(".yaml"):
    			mission_list += [file]

    	# for file in os.listdir(self.proprietary_mission_folder):
    	# 	if file.endswith(".mission"):
    	# 		mission_list += [file]

    	rospy.loginfo('[ZENO MISSION MANAGER FROM FILE] Mission List:')
    	print(mission_list)
    #---------------------------------------------------------------------------------------
    #---------------------------------------------------------------------------------------
    # callback meaning that the safety area of operation for the AUV has been loaded
    def load_mission_file_callback(self, msg):
        rospy.loginfo('[ZENO MISSION MANAGER FROM FILE] Loading filename [%s].', msg.data)
        if self.load_mission_file(msg.data):
            rospy.loginfo('[ZENO MISSION MANAGER FROM FILE] Mission file correctly loaded. It contains [%u] waypoints. Publishing it!', len(self.loaded_mission.waypoint_list))
            self.pub_mission.publish(self.loaded_mission)
            self.mission_loaded_pub.publish(msg.data)
        else:
            self.mission_loaded_pub.publish("ERROR")



#///////////////////////////////////////////////////////////////////////////////////////
#/////////////////////////////      MAIN         ///////////////////////////////////////
#///////////////////////////////////////////////////////////////////////////////////////
def main():
    rospy.init_node('zeno_mission_manager_from_file')

    # create the mission manager obj
    obj = Zeno_Mission_Manager_From_File()

    try:
        rospy.spin()
    except Exception:
        rospy.logfatal('[ZENO MISSION MANAGER FROM FILE] %s uncaught exception, dying!\n%s', __name__, traceback.format_exc())

if __name__ == '__main__':
    main()
