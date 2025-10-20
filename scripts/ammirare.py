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

json_dir  = rospy.get_param("/json_dir")
alpha_rot = rospy.get_param("/alpha_rot") # angolo fra NED e local

# Area di lavoro
A0 = np.array(rospy.get_param("/A0")) # vertice area di lavoro
A1 = np.array(rospy.get_param("/A1")) # vertice area di lavoro
A2 = np.array(rospy.get_param("/A2")) # vertice area di lavoro
A3 = np.array(rospy.get_param("/A3")) # vertice area di lavoro

# trasformazione vettori [latitudine, longitudine] in vettori [latitudine, longitudine, profondita]
A0 = np.array([ A0[0], A0[1], 0])
A1 = np.array([ A1[0], A1[1], 0])
A2 = np.array([ A2[0], A2[1], 0])
A3 = np.array([ A3[0], A3[1], 0])

# lista vertici dell' area di lavoro
A = np.array([A0, A1, A2, A3])

####################
#  CLASSE PLANNING #
####################

class planning:

    def __init__(self):
        rospy.init_node('planning', anonymous=True)

        # Pubblicazione waypoint
        self.pubUploadMission = rospy.Publisher("zeno/mm/upload_mission",WaypointList,queue_size=10)

        # Pubblicazione per stati del robot
        self.pubDeleteMission = rospy.Publisher("zeno/mm/delete_mission",Empty,queue_size=10)
        self.pubStartMission  = rospy.Publisher("zeno/mm/start_mission",Empty,queue_size=10)
        self.pubPauseMission  = rospy.Publisher("zeno/mm/pause_mission",Empty,queue_size=10)
        
        # Lettura stato della missione
        self.subZenoStatus = rospy.Subscriber("zeno/mm/mission_status", String, self.callback_ZenoStatus)

        # Lettura posizione
        self.subPosition = rospy.Subscriber("/nav_status",NavStatus,self.callback_pose)

        self.Mission = Empty() # definizione messaggio vuoto per le missioni

        # DEFINIZIONE ATTRIBUTI DELLA CLASSE
        self.rate               = rospy.Rate(10)     # frequenza di lavoro pari a 10Hz
        self.w_local            = []                 # lista che racchiude tutti i waypoint dati espressi in terna LOCAL
        self.w_trajectory_local = []                 # lista di waypoint locali
        self.w_trajectory       = []                 # lista di waypoint da pubblicare 
        self.wayList            = []                 # lista dei waypoint da pubblicare a zeno_mission_manager

        self.statusZeno         = ''                 # stringa che memorizza lo stato di missione del robot
        self.mission            = ''                 # tipo di missione

        self.latLongVector      = []                 # posizione di Zeno (vettore 3d)
        self.firstNavStatus     = True               # primo NavStatus ottenuto

        self.C_ne2lo = np.array(f.matrixNED2Local(alpha_rot))                           # matrice di rotazione NED2Local
        self.C_lo2ne = np.transpose(self.C_ne2lo)                                       # matrice di rotazione local2NED
        self.first_mission = True

        # Lettura json
        self.json_dir      = json_dir
        self.json          = None
        self.time          = 0.0
        self.bool_new_json = False
        self.force         = True

    ##########################################
                    # NODE #
    ##########################################

    def run(self):
        while not rospy.is_shutdown():

            if not self.firstNavStatus:

                # Check nuovo json
                self.WP, self.mission, self.json, self.time, self.bool_new_json = f.check_and_read_new_json(self.json_dir, self.json, self.time, self.force)
                self.force = False

                # Calcolo dell origine per il sistema locale (nel waypoint piu vicino)
                if self.bool_new_json:

                    print("-----------------")
                    for i in range(len(self.mission)):
                        print("Missione: ", self.mission[i], self.WP[i])
                    print("-----------------")
                        
                    ############## ESECUZIONE MISSIONE ##############
                    for i in range(len(self.mission)):
                        if self.mission[i] == "point":   
                            filtered_WP = f.filter_points_in_safety_area(self.WP[i], A, self.mission[i])
                            self.w_trajectory.extend(filtered_WP) 
                        else:
                            ordered_WP  = f.points_counterclockwise(self.WP[i]) 
                            filtered_WP = f.filter_points_in_safety_area(ordered_WP, A, self.mission[i])

                            dist_lld = []
                            for i in filtered_WP:
                                dist_lld.append(g.lld2distance(self.latLongVector, i))
                            min_index_lld = dist_lld.index(min(dist_lld))
                            NED_origin = filtered_WP[min_index_lld]

                            # per ogni waypoint in coordinate lld vengono calcolate le coordinate in terna LOCAL
                            for i in filtered_WP:
                                w_local_i = f.positionLocal(i,self.C_ne2lo,NED_origin)
                                self.w_local.append(w_local_i)

                            self.w_trajectory_local = f.transetti(self.w_local)
                            for e in self.w_trajectory_local:
                                self.w_trajectory.append(f.waypointLocal2LL(e,self.C_lo2ne,NED_origin))

                    self.wayList = f.waypointsList(self.w_trajectory)    # definizione del messaggio di waypoint
                                
                if self.statusZeno == "IDLE" and self.wayList:
                    self.pubUploadMission.publish(self.wayList)          # pubblicazione lista di waypoint per passare dallo stato IDLE a READY

                # condizione utile ad avviare la missione su Zeno aspettando che lo stato sia READY: lo stato diventera poi RUNNING
                if self.statusZeno == "READY":
                    self.wayList = []
                    self.pubStartMission.publish(self.Mission)

                if self.statusZeno == "COMPLETED":
                    self.pubDeleteMission.publish(self.Mission)

            self.rate.sleep()

    ##########################################
                  # CALLBACK #
    ##########################################

    # Callback relativa al NavsStatus
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
        print("Stato Zeno: " + self.statusZeno)   # viene stampato a schermo lo stato del robot

if __name__ == '__main__':
    try:
        node = planning()
        node.run()

    except rospy.ROSInterruptException: pass
