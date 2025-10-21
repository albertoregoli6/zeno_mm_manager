#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import String
from std_msgs.msg import Empty
from marta_msgs.msg import NavStatus
from zeno_mission_manager.msg import *

from library import geodetic_functions as g
from library import function as f

####################
# IMPORT PARAMETRI #
####################

json_dir  = rospy.get_param("/json_dir")    # cartella salvataggio json
alpha_rot = rospy.get_param("/alpha_rot")   # angolo fra NED e local

# Area di lavoro
A0 = np.array(rospy.get_param("/A0"))       # vertice area di lavoro
A1 = np.array(rospy.get_param("/A1"))       # vertice area di lavoro
A2 = np.array(rospy.get_param("/A2"))       # vertice area di lavoro
A3 = np.array(rospy.get_param("/A3"))       # vertice area di lavoro

A0 = np.array([ A0[0], A0[1], 0])
A1 = np.array([ A1[0], A1[1], 0])
A2 = np.array([ A2[0], A2[1], 0])
A3 = np.array([ A3[0], A3[1], 0])

# Lista vertci area di lavoro [lat, long, depth]
A = np.array([A0, A1, A2, A3])

####################
#  CLASSE PLANNING #
####################

class planning:

    def __init__(self):
        rospy.init_node('planning', anonymous=True)

        # Publisher lista waypoint
        self.pubUploadMission = rospy.Publisher("zeno/mm/upload_mission",WaypointList,queue_size=10)

        # Publishers per cambio stato missione di Zeno
        self.pubDeleteMission = rospy.Publisher("zeno/mm/delete_mission",Empty,queue_size=10)
        self.pubStartMission  = rospy.Publisher("zeno/mm/start_mission",Empty,queue_size=10)
        self.pubPauseMission  = rospy.Publisher("zeno/mm/pause_mission",Empty,queue_size=10)
        
        # Subscriber stato della missione
        self.subZenoStatus = rospy.Subscriber("zeno/mm/mission_status", String, self.callback_ZenoStatus)

        # Subscriber nav_status Zeno
        self.subPosition = rospy.Subscriber("/nav_status",NavStatus,self.callback_pose)

        # Messaggio per cambio stato missione
        self.Mission = Empty() 

        # Inizializzazione variabili della classe
        self.rate               = rospy.Rate(10)                # frequenza di lavoro pari a 10Hz
        self.w_local            = []                            # lista che racchiude tutti i waypoint ricevuti espressi in terna LOCAL
        self.w_trajectory_local = []                            # lista di waypoint locali da dare a Zeno
        self.w_trajectory       = []                            # lista di waypoint da pubblicare a Zeno
        self.wayList            = []                            # messaggio ROS della lista dei waypoint da pubblicare a zeno_mission_manager

        self.statusZeno         = ''                            # stringa che memorizza lo stato di missione del robot
        self.mission            = ''                            # tipo di missione (transetti o point)

        self.latLongVector      = []                            # posizione di Zeno (vettore 3d)
        self.firstNavStatus     = True                          # primo NavStatus ottenuto

        self.C_ne2lo = np.array(f.matrixNED2Local(alpha_rot))   # matrice di rotazione NED2Local
        self.C_lo2ne = np.transpose(self.C_ne2lo)               # matrice di rotazione local2NED
        self.first_mission = True

        # Lettura json
        self.json_dir      = json_dir                           # path cartella salvataggio dei file .json
        self.json          = None                               # nome del file .json
        self.time          = 0.0                                # tempo 
        self.bool_new_json = False                              # booleano per indicare presenza o no di un nuovo file .json
        self.force         = True                               # variabile per inizializzare il check dei nuovi file .json

    ##########################################
                    # NODE #
    ##########################################

    def run(self):
        while not rospy.is_shutdown():

            if not self.firstNavStatus:

                # Check nuovo json
                self.WP, self.mission, self.json, self.time, self.bool_new_json = f.check_and_read_new_json(self.json_dir, self.json, self.time, self.force)
                self.force = False

                # Calcolo dei waypoint della missione
                if self.bool_new_json:

                    print("-----------------")
                    for i in range(len(self.mission)):
                        print("Missione: ", self.mission[i], self.WP[i])
                    print("-----------------")
                        
                    # Costruzione waypoint (per il caso point e per i transetti)
                    for i in range(len(self.mission)):
                        if self.mission[i] == "point":   
                            # Check waypoint dentro la safety area
                            filtered_WP = f.filter_points_in_safety_area(self.WP[i], A, self.mission[i])
                            self.w_trajectory.extend(filtered_WP) 
                        else:
                            # Ordinamento in senso antiorario dei waypoint e check dentro la safety area
                            ordered_WP  = f.points_counterclockwise(self.WP[i]) 
                            filtered_WP = f.filter_points_in_safety_area(ordered_WP, A, self.mission[i])

                            # Calcolo dei transetti
                            dist_lld = []
                            for i in filtered_WP:
                                dist_lld.append(g.lld2distance(self.latLongVector, i))
                            min_index_lld = dist_lld.index(min(dist_lld))
                            NED_origin = filtered_WP[min_index_lld]

                            for i in filtered_WP:
                                w_local_i = f.positionLocal(i,self.C_ne2lo,NED_origin)
                                self.w_local.append(w_local_i)

                            self.w_trajectory_local = f.transetti(self.w_local)
                            for e in self.w_trajectory_local:
                                self.w_trajectory.append(f.waypointLocal2LL(e,self.C_lo2ne,NED_origin))

                    # Definizione del messaggio ROS 
                    self.wayList = f.waypointsList(self.w_trajectory)
                    self.bool_new_json = False

                # Pubblicazione lista di waypoint per passare dallo stato IDLE a READY           
                if self.statusZeno == "IDLE" and self.wayList:
                    self.pubUploadMission.publish(self.wayList)          

                # Pubblicazione del messaggio vuoto per passare da READY a RUNNING
                if self.statusZeno == "READY":
                    self.w_trajectory = []
                    self.wayList      = []
                    self.pubStartMission.publish(self.Mission)

                # Pubblicazione del messaggio vuoto per passare da COMPLETED a IDLE se presente un nuovo file .json
                if self.statusZeno == "COMPLETED" and self.wayList:
                    self.pubDeleteMission.publish(self.Mission)

            self.rate.sleep()

    ##########################################
                  # CALLBACK #
    ##########################################

    # Callback relativa al Nav_Status
    def callback_pose(self,navStatus_msg):
        # posizione del robot in terna LLD (latitudine, longitudine, depth)
        latitude           = navStatus_msg.position.latitude                          # estrazione latitudine ECEF
        longitudine        = navStatus_msg.position.longitude                         # estrazione longitudine ECEF
        altitude           = navStatus_msg.position.depth                             # estrazione profondita ECEF
        self.orientation   = navStatus_msg.orientation.yaw                            # orientazione NED
        self.latLongVector = np.array([latitude, longitudine, altitude])              # composizione delle 3 variabili in terna LL

        if self.firstNavStatus:
            self.firstNavStatus = False
            print("PRIMO GPS")

    # Callback dello stato di missione di Zeno:
    def callback_ZenoStatus(self, msgStatus):
        self.statusZeno = msgStatus.data
        print("Stato Zeno: " + self.statusZeno)

if __name__ == '__main__':
    try:
        node = planning()
        node.run()

    except rospy.ROSInterruptException: pass