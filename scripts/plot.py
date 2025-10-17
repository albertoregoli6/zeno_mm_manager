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

# Area di lavoro
A0 = np.array(rospy.get_param("/A0"))
A1 = np.array(rospy.get_param("/A1"))
A2 = np.array(rospy.get_param("/A2"))
A3 = np.array(rospy.get_param("/A3"))
A  = np.array([A0, A1, A2, A3])   # shape (4,2) o (4,3)

####################
#  CLASSE PLANNING #
####################

class plot:

    def __init__(self):
        rospy.init_node('plot', anonymous=True)

        # Sorgenti dati
        self.sub_nav     = rospy.Subscriber("/nav_status", NavStatus, self.callback_pose)
        self.sub_waylist = rospy.Subscriber("/zeno/mm/upload_mission", WaypointList, self.waypoint_callback)

        plt.ion()

        self.rate          = rospy.Rate(2)
        self.latLongVector = []              
        self.waypoint_list = []              

        # Missione da file JSON
        self.WP, self.mission = f.read_points_from_json("/home/alberto/zeno_ws/src/zeno_mission_manager/json/niasca.json")

    def run(self):
        while not rospy.is_shutdown():
            # Converte A in lista di liste per il plot
            A_list = A[:, :2].tolist() if isinstance(A, np.ndarray) else A

            # latLongVector: np.array([lat, lon]) -> [[lat, lon]]
            if isinstance(self.latLongVector, np.ndarray) and self.latLongVector.size >= 2:
                zeno_point = [[float(self.latLongVector[0]), float(self.latLongVector[1])]]
            elif isinstance(self.latLongVector, (list, tuple)) and len(self.latLongVector) >= 2:
                zeno_point = [[float(self.latLongVector[0]), float(self.latLongVector[1])]]
            else:
                zeno_point = []

            # Plot
            f.plot_mission(A_list,self.waypoint_list,self.WP,zeno_point,self.mission)

            self.rate.sleep()

    # ================== CALLBACKS ==================

    def callback_pose(self, navStatus_msg):
        # posizione robot in LLD
        latitude    = navStatus_msg.position.latitude
        longitude   = navStatus_msg.position.longitude
        self.latLongVector = np.array([latitude, longitude], dtype=float)

    def waypoint_callback(self, waypoint_msg):
        # estrai solo lat/lon in [[lat,lon], ...]
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
