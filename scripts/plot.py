#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from library import function as f
from marta_msgs.msg import NavStatus
from zeno_mission_manager.msg import WaypointList

####################
# IMPORT PARAMETRI #
####################

# Cartella salvataggio json
json_dir  = rospy.get_param("/json_dir")

# Area di lavoro
A0 = np.array(rospy.get_param("/A0"))
A1 = np.array(rospy.get_param("/A1"))
A2 = np.array(rospy.get_param("/A2"))
A3 = np.array(rospy.get_param("/A3"))
A  = np.array([A0, A1, A2, A3])

#################
#  CLASSE PLOT  #
#################

class plot:

    def __init__(self):
        rospy.init_node('plot', anonymous=True)

        # Subscriber
        self.sub_nav     = rospy.Subscriber("/nav_status", NavStatus, self.callback_pose)
        self.sub_waylist = rospy.Subscriber("/zeno/mm/upload_mission", WaypointList, self.waypoint_callback)

        plt.ion()

        self.rate          = rospy.Rate(2)
        self.latLongVector = []              
        self.waypoint_list = []     

        # Lettura json
        self.json_dir      = json_dir
        self.json          = None
        self.time          = 0.0
        self.bool_new_json = False
        self.force         = True         

    def run(self):
        while not rospy.is_shutdown():
            # Lista vertici safety area
            A_list = A[:, :2].tolist() if isinstance(A, np.ndarray) else A

            # Posizione lat-long di Zeno
            if isinstance(self.latLongVector, np.ndarray) and self.latLongVector.size >= 2:
                zeno_point = [[float(self.latLongVector[0]), float(self.latLongVector[1])]]
            elif isinstance(self.latLongVector, (list, tuple)) and len(self.latLongVector) >= 2:
                zeno_point = [[float(self.latLongVector[0]), float(self.latLongVector[1])]]
            else:
                zeno_point = []

            # Check nuovo json
            WP, mission, self.json, self.time, self.bool_new_json = f.check_and_read_new_json(self.json_dir, self.json, self.time, self.force)
            self.force = False

            if self.bool_new_json:
                self.WP            = WP
                self.mission       = mission
                self.bool_new_json = False

            # Creazione del plot
            f.plot_mission(A_list,self.waypoint_list,self.WP,zeno_point,self.mission)

            self.rate.sleep()

    ##########################################
                  # CALLBACK #
    ##########################################

    # Callback relativa al Nav_Status
    def callback_pose(self, navStatus_msg):
        # posizione robot in LL
        latitude    = navStatus_msg.position.latitude
        longitude   = navStatus_msg.position.longitude
        self.latLongVector = np.array([latitude, longitude], dtype=float)

    # Callback waypoint pubblicati a Zeno
    def waypoint_callback(self, waypoint_msg):
        pts = []
        for wp in waypoint_msg.waypoint_list:
            if hasattr(wp, 'position'):
                if hasattr(wp.position, 'latitude') and hasattr(wp.position, 'longitude'):
                    pts.append([float(wp.position.latitude), float(wp.position.longitude)])
        self.waypoint_list = pts

if __name__ == '__main__':
    try:
        node = plot()
        node.run()
    except rospy.ROSInterruptException:
        pass
