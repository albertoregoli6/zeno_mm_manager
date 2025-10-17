#!/usr/bin/env python
import rospy
import numpy as np
import math
from std_msgs.msg import String
from marta_msgs.msg import NavStatus
from zeno_mission_manager.msg import *
from std_msgs.msg import Empty
from library import geodetic_functions as g
from library import function as f

from geometry_msgs.msg import Point

##################################
# INIZIALIZZAZIONE DEI PARAMETRI #
##################################

wayList    = [] # lista dei waypoint definiti tramite il pacchetto di zeno_mission_manager che verranno pubblicati
NED_origin = np.array([0,0,0])

####################
# IMPORT PARAMETRI #
####################

Olocal    = np.array(rospy.get_param("/Olocal")) # origine terna LOCAL in NED
alpha_rot = rospy.get_param("/alpha_rot") # angolo fra NED e local
mission   = rospy.get_param("/mission")

# Area di lavoro
WP0 = np.array(rospy.get_param("/WP0")) # vertice area di lavoro
WP1 = np.array(rospy.get_param("/WP1")) # vertice area di lavoro
WP2 = np.array(rospy.get_param("/WP2")) # vertice area di lavoro
WP3 = np.array(rospy.get_param("/WP3")) # vertice area di lavoro

# trasformazione vettori [latitudine, longitudine] in vettori [latitudine, longitudine, profondita]
WP0 = np.array([ WP0[0], WP0[1], 0])
WP1 = np.array([ WP1[0], WP1[1], 0])
WP2 = np.array([ WP2[0], WP2[1], 0])
WP3 = np.array([ WP3[0], WP3[1], 0])

# lista vertici dell' area di lavoro
WP = np.array([WP0, WP1,WP2, WP3])

####################
#  CLASSE PLANNING #
####################

class planning:
    """ La classe planning rappresenta il nodo che verra eseguito e al suo interno contiene:
         - 5 Publisher che pubblicano i waypoint per la missione in base a dove si trova il robot e cambiano lo stato (idle,ready,running,pause);
         - 2 Subscriber che leggono rispettivamente lo stato della missione e lo stato della navigazione del robot. """

    def __init__(self):
        # Pubblicazione waypoint
        self.pubUploadMission = rospy.Publisher("zeno/mm/upload_mission",WaypointList,queue_size=10)

        # Pubblicazione per stati del robot
        self.pubDeleteMission = rospy.Publisher("zeno/mm/delete_mission",Empty,queue_size=10)
        self.pubStartMission = rospy.Publisher("zeno/mm/start_mission",Empty,queue_size=10)
        self.pubPauseMission = rospy.Publisher("zeno/mm/pause_mission",Empty,queue_size=10)

        # Pubblicazione waypoint per la visualizzazione in Plotjuggler
        self.pubPosWayZeno = rospy.Publisher("/posWay",Point,queue_size=10) 
        
        # Lettura stato della missione
        self.subZenoStatus = rospy.Subscriber("zeno/mm/mission_status", String, self.callback_ZenoStatus)

        # Lettura posizione
        self.subPosition = rospy.Subscriber("/nav_status",NavStatus,self.callback_pose)

        self.Mission = Empty() # definizione messaggio vuoto per le missioni

        # DEFINIZIONE ATTRIBUTI DELLA CLASSE
        self.rate         = rospy.Rate(10)          # frequenza di lavoro pari a 10Hz
        self.primoPasso   = True                    # variabile booleana che serve per il calcolo iniziale del vertice piu vicino al robot: se False significa che ha iniziato la naviagazione
        self.initWaypoint = True                    # variabile booleana usata per il calcolo in terna local dei primi 9 waypoint: se False significa che i waypoint iniziali sono stati caricati
        self.P_local      = []                      # lista che racchiude le coordinate della posizione del robot
        self.w_local      = []                      # lista che racchiude tutti i waypoint espressi in terna LOCAL
        self.C_ne2lo      = []                      # matrice di rotazione NED2Local
        self.orientation  = 0                       # intero che definisce l'orientazione del robot
        self.w_trajectory = []                      # lista di waypoint da pubblicare 
        self.C_lo2ne      = []                      # matrice di rotazione LOCAL2Ned
        self.statusZeno   = ''                      # stringa che memorizza lo stato del robot

        rospy.loginfo("Starting planning...")

    ##########################################
                  # CALLBACK #
    ##########################################

    # Callback relativa al subscriber dello stato di navigazione:
    # TOPIC: /nav_status
    # MESSAGGIO: NavStatus
    def callback_pose(self,navStatus_msg):
        global wayList
        global mission
        global NED_origin

        # posizione del robot in terna LLD (latitudine, longitudine, depth)
        latitude         = navStatus_msg.position.latitude                          # estrazione latitudine ECEF
        longitudine      = navStatus_msg.position.longitude                         # estrazione longitudine ECEF
        altitude         = navStatus_msg.position.depth                             # estrazione profondita ECEF
        self.orientation = navStatus_msg.orientation.yaw                            # orientazione NED
        latLongVector    = np.array([latitude, longitudine, altitude])              # composizione delle 3 variabili in terna LL

        # Calcolo dell origine per il sistema locale (nel waypoint piu vicino)
        if self.initWaypoint:
            dist_lld = []
            for i in WP:
                dist_lld.append(g.lld2distance(latLongVector, i))
            min_index_lld = dist_lld.index(min(dist_lld))
            NED_origin = WP[min_index_lld]
        
        # Calcolo della posizione di zeno nella terna locale
        self.C_ne2lo = np.array(f.matrixNED2Local(alpha_rot))                           # matrice di rotazione NED2Local
        self.C_lo2ne = np.transpose(self.C_ne2lo)                                       # matrice di rotazione local2NED
        self.P_local = np.array(f.positionLocal(latLongVector,self.C_ne2lo,NED_origin)) # posizione corrente del robot

        if self.initWaypoint and self.statusZeno=="IDLE" and not len(self.P_local)==0:
            rospy.loginfo("INDICE ORIGINE NED: {} | NED_origin: {}".format(min_index_lld, NED_origin))
            # per ogni waypoint in coordinate lld vengono calcolate le coordinate in terna LOCAL
            for i in WP:
                w_local_i = f.positionLocal(i,self.C_ne2lo,NED_origin)
                self.w_local.append(w_local_i)

            if mission == "transetti":   
                self.w_trajectory = f.transetti(self.w_local)
            else: 
                minIndex,self.w_trajectory = f.diagonali(self.w_local,self.P_local)
                print("Casistica: {data}".format(data=(minIndex, self.w_trajectory)))

            #rospy.logwarn("w_trajectory: {}".format(self.w_trajectory))
            wayList = f.waypointsList(self.w_trajectory,self.C_lo2ne,NED_origin)    # definizione del messaggio di waypoint
            self.pubUploadMission.publish(wayList)                                  # viene caricata la lista di waypoint che permette al robot di passare dallo stato IDLE a READY
            rospy.logwarn("WAY_LIST: {}".format(wayList))
            wayList = []                                                            # serve per svuotare la lista di waypoint 

        # condizione utile ad avviare la missione su Zeno aspettando che lo stato sia READY: lo stato diventera poi RUNNING
        if self.initWaypoint and self.statusZeno == "READY":
            self.pubStartMission.publish(self.Mission)
            self.primoPasso   = False                                               # viene messa a False in modo da pubblicare i waypoint solo una volta
            self.initWaypoint = False                                               # viene messa a False perche i primi 9 waypoint sono stati pubblicati

        ############## PLOTJUGGLER ##############
        # definizione del messaggio per la visualizzazione su plotjuggler
        poszenoWay   = Point()
        poszenoWay.x = float(self.P_local[0])
        poszenoWay.y = float(self.P_local[1])
        poszenoWay.z = float(self.P_local[2])
        self.pubPosWayZeno.publish(poszenoWay)


    # Callback relativa al subscriber dello stato di navigazione:
    # TOPIC: zeno/mm/mission_status
    # MESSAGGIO: String
    # callback che restituisce lo stato del robot(idle, ready, paused, running, completed)
    def callback_ZenoStatus(self, msgStatus):
        self.statusZeno = msgStatus.data
        print("Stato Zeno: " + self.statusZeno)   # viene stampato a schermo lo stato del robot

if __name__ == '__main__':
    rospy.init_node('planning')
    planning()
    rospy.spin()
