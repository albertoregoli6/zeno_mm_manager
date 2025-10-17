#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
import json
import os
import numpy as np
import matplotlib.pyplot as plt
from zeno_mission_manager.msg import *
import geodetic_functions

####################
# IMPORT PARAMETRI #
####################

Olocal    = np.array(rospy.get_param("/Olocal")) # origine terna LOCAL in NED
alpha_rot = rospy.get_param("/alpha_rot") # angolo fra NED e local
step      = rospy.get_param("/step") # spaziatura transetti

##################
# FUNZIONI UTILI #
##################

# funzione che restituisce la matrice di rotazione C_ne2lo da NED a Local
def matrixNED2Local(alpha_rot): 
    C_ne2lo = [[math.cos(alpha_rot), math.sin(alpha_rot), 0],[-math.sin(alpha_rot), math.cos(alpha_rot), 0],[0, 0, 1]]
    return C_ne2lo

# funzione che restituisce il punto in terna Local dato il punto in terna ECEF
def positionLocal(latLongVector,C_ne2lo,NED_origin):
    P_ll2ned = np.array(geodetic_functions.lld2ned(NED_origin,latLongVector))  
    P_local = np.dot(C_ne2lo, P_ll2ned) - np.dot(C_ne2lo, Olocal)
    return P_local.tolist()

# funzione che dato un waypoint in terna local, restituisce in componenti in terna ECEF: [latitudine,longitudine]
def waypointLocal2LL(w_local,C_lo2ne,NED_origin):
    w_ned = np.array(np.dot(C_lo2ne,w_local) + Olocal) 
    W_ned2ll = np.array(geodetic_functions.ned2lld(NED_origin,w_ned)) 
    # w_ned2ll: punto nella terna ECEF del waypoint
    return W_ned2ll

# funzione che restituisce il messaggio corretto per i waypoints
def waypoints(w_ned2ll):
    print(w_ned2ll)
    # way e il waypoint i-esimo presente nel messaggio Waypoint() che viene pubblicato durante la missione
    way = Waypoint() 
    way.position.latitude = w_ned2ll[0]
    way.position.longitude = w_ned2ll[1]
    way.position.depth = 0.0
    way.speed = 0.5 # m/s
    way.control_mode = "depth"
    way.altitude = 0.0
    return way

# funzione che ci restituisce i waypoint in terna ll (da terna local), prendendo in ingresso il vettore dei waypoints che il robot deve raggiungere 
# in terna local (w_trajectory). Tale funzione produce la lista dei waypoints in terna ll da inserire nel messaggio WaypoinList.
# Inoltre necessita della matrice di rotazione tra la terna local e la terna ned.
def waypointsList(w_trajectory):
    wayList = WaypointList()
    for i in w_trajectory:
        way = waypoints(i)
        wayList.waypoint_list.append(way)
        print("waypoint: {}".format(i))
    return wayList

###################################################################
####################### ALGORITMO DIAGONALI #######################
###################################################################

# funzione usata per trovare il vertice piu vicino da cui partire. In ingresso necessita del vettore dei vertici dell'area di lavoro in componente in terna local 
# e del vettore posizione del robot in terna local
def min_distance(vertices,P_local):
    d = []
    # per ogni vertice viene calcolata la distanza dello stesso dal robot e si definisce l'indice (min_index) che permette di estrarre il vertice a distanza minima dal robot
    for i in vertices:
        d.append(math.sqrt((P_local[0]-i[0])**2 + (P_local[1]-i[1])**2))
    min_index = d.index(min(d))
    print(min_index)
    return min_index

# la funzione produce la lista dei waypoints ordinati in terna local per tutti gli scenari possibili, ovvero
# al variare del primo waypoint [way1], definito come il vertice piu vicino a Zeno al tempo zero. 
# w_localTotal e il vettore dei waypoint in terna local utilizzati per la fase di esplorazione
def switch(way1, w_localTotal):
    w_trajectory = []
    if np.all(way1 == w_localTotal[0]):
        w_trajectory = [w_localTotal[0], w_localTotal[2], w_localTotal[3], w_localTotal[1], w_localTotal[4], w_localTotal[7], w_localTotal[6], w_localTotal[5], w_localTotal[4]]
    elif np.all(way1 == w_localTotal[1]):
        w_trajectory = [w_localTotal[1], w_localTotal[3], w_localTotal[2],  w_localTotal[0], w_localTotal[4], w_localTotal[5], w_localTotal[6], w_localTotal[7], w_localTotal[4]]
    elif np.all(way1 == w_localTotal[2]):
        w_trajectory = [w_localTotal[2], w_localTotal[0], w_localTotal[1], w_localTotal[3], w_localTotal[6], w_localTotal[5], w_localTotal[4], w_localTotal[7], w_localTotal[6]]
    elif np.all(way1 == w_localTotal[3]):
        w_trajectory = [w_localTotal[3], w_localTotal[1], w_localTotal[0], w_localTotal[2], w_localTotal[6], w_localTotal[7], w_localTotal[4], w_localTotal[5], w_localTotal[6]]
    return np.array(w_trajectory)

def diagonali(w_local,P_local):
    # definizione waypoint in terna local
    # waypoint per la costruzione della clessidra
    w_local_0 = w_local[0]                                             # waypoint 0 in terna local
    w_local_1 = w_local[1]                                             # waypoint 1 in terna local
    w_local_2 = w_local[2]                                             # waypoint 2 in terna local
    w_local_3 = w_local[3]                                             # waypoint 3 in terna local

    # waypoint per la costruzione del rombo (rotazione antioraria)
    # waypoint in terna local posizionati nel punto medio dei lati del poligono dell'area di navigazione 
    w_local_4 = 0.5*(w_local_0 + w_local_1) 
    w_local_5 = 0.5*(w_local_1 + w_local_2)
    w_local_6 = 0.5*(w_local_2 + w_local_3)
    w_local_7 = 0.5*(w_local_3 + w_local_0)

    # costruzione vettore di waypoint
    w_localTotal = [w_local_0, w_local_1, w_local_2,w_local_3, w_local_4, w_local_5, w_local_6, w_local_7] # qui ci vanno tutti i waypoint

    # funzione che viene eseguita una sola volta per trovare il vertice iniziale piu vicino 
    minIndex     = min_distance(w_localTotal[0:4], P_local)
    way1         = w_localTotal[minIndex]                               # primo waypoint in cui si posiziona il robot
    w_trajectory = switch(way1, w_localTotal)                           # funzione switch: per scegliere lo scenario iniziale di navigazione
    return minIndex, w_trajectory

###################################################################
####################### ALGORITMO TRANSETTI #######################
###################################################################

def points_counterclockwise(points):
    lats = [p[0] for p in points]
    lons = [p[1] for p in points]

    # calcola il centroide
    centroid_lat = sum(lats) / len(points)
    centroid_lon = sum(lons) / len(points)

    # funzione per calcolare l'angolo di ciascun punto rispetto al centroide
    def angle_from_centroid(point):
        lat, lon = point[0], point[1]
        return math.atan2(lat - centroid_lat, lon - centroid_lon)

    # ordina i punti in base all'angolo (antiorario)
    sorted_points = sorted(points, key=angle_from_centroid, reverse=False)

    return sorted_points

def seg_intersection(ax, ay, bx, by, cx, cy, dx, dy):
    den = (bx - ax) * (dy - cy) - (by - ay) * (dx - cx)
    if abs(den) < 1e-12:
        return (False, 0.0, 0.0, 0.0)  # paralleli o quasi
    t = ((cx - ax) * (dy - cy) - (cy - ay) * (dx - cx)) / den  # su AB
    u = ((cx - ax) * (by - ay) - (cy - ay) * (bx - ax)) / den  # su CD
    if -1e-12 <= t <= 1.0 + 1e-12 and -1e-12 <= u <= 1.0 + 1e-12:
        x = ax + t * (bx - ax)
        y = ay + t * (by - ay)
        return (True, x, y, t)
    return (False, 0.0, 0.0, 0.0)

def interp_depth(d1, d2, t):
    if d1 is None or d2 is None:
        return None
    try:
        return (1.0 - t) * float(d1) + t * float(d2)
    except:
        return None

def add_unique_intersection(L, lat, lon, dep):
    """Evita duplicati numerici di intersezioni (tangenti/vertici)."""
    for q in L:
        if abs(q[0] - lat) < 1e-10 and abs(q[1] - lon) < 1e-10:
            return
    L.append((lat, lon, dep))

def filter_points_in_safety_area(points, safety_area, mission):
    n = len(safety_area)

    # Estrazione (lat,lon) del poligono
    poly = []
    for v in safety_area:
        lat = float(v[0])
        lon = float(v[1])
        poly.append((lat, lon))

    inside_points = []
    eps = 1e-12

    m = len(points)
    for idx, p in enumerate(points):
        # Punto corrente
        p_lat = float(p[0])
        p_lon = float(p[1])
        p_dep = p[2] if len(p) >= 3 else None

        # Ray casting + controllo bordo
        inside = False
        j = n - 1
        on_edge = False

        for i in range(n):
            yi = poly[i][0]; xi = poly[i][1]   # (lat, lon)
            yj = poly[j][0]; xj = poly[j][1]

            # Test punto sul bordo del segmento (i-j) tramite area del parallelogramma
            area = (p_lon - xi) * (yj - yi) - (p_lat - yi) * (xj - xi)
            if abs(area) <= eps and min(xi, xj) - eps <= p_lon <= max(xi, xj) + eps and min(yi, yj) - eps <= p_lat <= max(yi, yj) + eps:
                on_edge = True
                break

            # Ray casting: intersezione con riga orizzontale y = p_lat
            yi_gt = yi > p_lat
            yj_gt = yj > p_lat
            if yi_gt != yj_gt:
                denom = (yj - yi)
                if abs(denom) > eps:
                    x_int = xi + (xj - xi) * (p_lat - yi) / denom
                    if x_int >= p_lon - eps:
                        inside = not inside

            j = i

        if on_edge or inside:
            # Assicura tripla (lat, lon, depth) in uscita
            if len(p) >= 3:
                inside_points.append([p[0], p[1], p[2]])
            else:
                inside_points.append([p[0], p[1], None])

        elif (not on_edge) and (not inside) and mission == "transetti":
            # Punto esterno: cerca intersezioni con i lati del poligono di sicurezza
            inters = []

            # Segmenti del soggetto: prev->p e p->next (con wrap-around)
            prev_idx = (idx - 1) % m
            next_idx = (idx + 1) % m
            prev_pt = points[prev_idx]
            next_pt = points[next_idx]

            # Prev -> P
            a_lat = float(prev_pt[0])
            a_lon = float(prev_pt[1])
            a_dep = prev_pt[2] if len(prev_pt) >= 3 else None
            b_lat = p_lat   
            b_lon = p_lon
            b_dep = p_dep

            # P -> Next
            c_lat = p_lat;            c_lon = p_lon
            c_dep = p_dep
            d_lat = float(next_pt[0]); d_lon = float(next_pt[1])
            d_dep = next_pt[2] if len(next_pt) >= 3 else None

            # Contro tutti i lati del poligono di sicurezza
            for k in range(n):
                y1 = poly[k][0]; x1 = poly[k][1]
                y2 = poly[(k+1) % n][0]; x2 = poly[(k+1) % n][1]

                # Intersezione con prev->p
                ok1, ix1, iy1, t1 = seg_intersection(a_lon, a_lat, b_lon, b_lat, x1, y1, x2, y2)
                if ok1:
                    dep1 = interp_depth(a_dep, b_dep, t1)
                    add_unique_intersection(inters, iy1, ix1, dep1)

                # Intersezione con p->next
                ok2, ix2, iy2, t2 = seg_intersection(c_lon, c_lat, d_lon, d_lat, x1, y1, x2, y2)
                if ok2:
                    dep2 = interp_depth(c_dep, d_dep, t2)
                    add_unique_intersection(inters, iy2, ix2, dep2)

            # Aggiungi tutte le intersezioni trovate (0,1,2)
            for v in inters:
                inside_points.append(list(v))

    if len(inside_points) == 0:
        return [list(v) for v in safety_area]

    return [list(v) for v in inside_points]

def is_point_inside_polygon(x, y, area_lines):
    intersections = 0

    for m_a, q_a in area_lines:
        # Metodo del raggio per determinare se dentro
        if m_a == float('inf'):  
            x_line = q_a
            if x < x_line:
                intersections += 1
        else:
            x_intersect = (y - q_a) / m_a if m_a != 0 else None
            if x_intersect is not None and x <= x_intersect:
                intersections += 1

    return intersections % 2 == 1

def add_unique(pt_list, x, y, z=0.0, tol=1e-10):
    for px, py, pz in pt_list:
        if abs(px - x) < tol and abs(py - y) < tol:
            return
    pt_list.append((x, y, z))

def intersect_line_with_segment(m_t, q_t, p1, p2, eps=1e-12):
    x1, y1 = float(p1[0]), float(p1[1])
    x2, y2 = float(p2[0]), float(p2[1])

    # Segmento verticale?
    seg_vertical = abs(x2 - x1) <= eps

    if m_t == float('inf'):
        # retta verticale: x = q_t
        x = q_t
        if seg_vertical:
            # paralleli verticali: collineari o disgiunti -> nessuna intersezione puntuale
            if abs(x1 - x) <= eps:
                return (False, 0.0, 0.0)  # collineare: ignora (infinite punti)
            else:
                return (False, 0.0, 0.0)  # parallelo disgiunto
        else:
            m_s = (y2 - y1) / (x2 - x1)
            q_s = y1 - m_s * x1
            y = m_s * x + q_s
    else:
        # retta non-verticale: y = m_t x + q_t
        if seg_vertical:
            # x = x1
            x = x1
            y = m_t * x + q_t
        else:
            m_s = (y2 - y1) / (x2 - x1)
            q_s = y1 - m_s * x1
            # rette parallele?
            if abs(m_s - m_t) <= eps:
                # collineari o parallele disgiunte: nessuna intersezione puntuale
                if abs((m_t * x1 + q_t) - y1) <= eps:
                    return (False, 0.0, 0.0)  # collineare: ignora
                else:
                    return (False, 0.0, 0.0)
            # m_t x + q_t = m_s x + q_s  ->  x = (q_s - q_t)/(m_t - m_s)
            x = (q_s - q_t) / (m_t - m_s)
            y = m_t * x + q_t

    # Controllo che (x,y) sia DENTRO il segmento (bounding box con tolleranza)
    if (min(x1, x2) - eps <= x <= max(x1, x2) + eps) and (min(y1, y2) - eps <= y <= max(y1, y2) + eps):
        return (True, x, y)
    return (False, 0.0, 0.0)

def transetti(w_local):

    #rospy.logwarn("=== INIZIO TRANSETTI ===")
    #rospy.logwarn("Input w_local: {}".format(w_local))

    # Trova l'indice dell'origine [0,0,0] in w_local
    idx0 = 0
    for i in range(len(w_local)):
        if w_local[i][0] == 0.0 and w_local[i][1] == 0.0:
            idx0 = i
            break

    # Indici precedente e successivo a NED_origin
    npts = len(w_local)
    idx_prev = (idx0 - 1) % npts
    idx_next = (idx0 + 1) % npts

    # Distanze planari (x,y) da [0,0,0] a prev/next
    dist_prev = math.sqrt((w_local[idx0][0]-w_local[idx_prev][0])**2 + (w_local[idx0][1]-w_local[idx_prev][1])**2 + (w_local[idx0][2]-w_local[idx_prev][2])**2)
    dist_next = math.sqrt((w_local[idx0][0]-w_local[idx_next][0])**2 + (w_local[idx0][1]-w_local[idx_next][1])**2 + (w_local[idx0][2]-w_local[idx_next][2])**2)

    # Scegli il punto con distanza maggiore e ottieni l'indice
    idx_2_min = idx_prev if dist_prev >= dist_next else idx_next

    # Calcolo del coefficiente angolare e dell intercetta
    retta_transetti =[]
    x1, y1 = w_local[idx0][0], w_local[idx0][1]
    x2, y2 = w_local[idx_2_min][0], w_local[idx_2_min][1]
    if abs(x2 - x1) > 1e-12:
        m = (y2 - y1) / (x2 - x1)
        q = y1 - m * x1
        retta_transetti.extend([m, q])
        #rospy.logwarn("Retta principale: m={} | q={}".format(m, q))
    else:
        m = float('inf')   # retta verticale: x = x1
        q = None
        retta_transetti.append(m)
        #rospy.logwarn("Retta principale: verticale (x = {})".format(x1))

    #print("---------------------------------------")

    # Punto più lontano dalla retta dei transetti
    max_dist = -1.0
    idx_max  = 0
    if m != float('inf'):
        denom = math.sqrt(m*m + 1.0)
        for i, p in enumerate(w_local):
            xi, yi = float(p[0]), float(p[1])
            d = abs(m*xi - yi + q) / denom   # distanza punto-retta (forma implicita)
            if d > max_dist:
                max_dist = d
                idx_max  = i
    else:
        # retta verticale: distanza = |x - x1|
        for i, p in enumerate(w_local):
            xi = float(p[0])
            d = abs(xi - x1)
            if d > max_dist:
                max_dist = d
                idx_max  = i

    # Numero dei transetti
    num_transetti = int(math.ceil(max_dist / float(step))) + 1 if max_dist > 0 else 1
    #rospy.logwarn("Numero transetti (n_lm): {}, distanza max {}".format(num_transetti, max_dist))
    #print("---------------------------------------")

    # Direzione di sviluppo dei transetti
    if m != float('inf'):
        y_line = m * w_local[idx_max][0] + q
        direction_factor = 1 if w_local[idx_max][1] >= y_line else -1  # Sopra: 1, Sotto: -1
    else:
        direction_factor = 1 if w_local[idx_max][0] >= q else -1

    #rospy.logwarn("direction_factor: {}".format(direction_factor))
    #print("---------------------------------------")

    # Calcolo delle rette parallele distanziate di 'step' e tutte dallo stesso lato del punto idx_max
    transetti = []

    if m != float('inf'):
        transetti.append([m, q + 0.5*step])  # prima retta: quella trovata traslata di 0.5*step

        # Incremento in direzione perpendicolare
        dq = float(step) * math.sqrt(m*m + 1.0)

        for i in range(1, num_transetti):
            new_q = q + 0.5*step + direction_factor * i * dq
            transetti.append([m, new_q])
    else:
        x0 = x1  # retta base: x = x1
        transetti.append([float('inf'), x0])

        for i in range(1, num_transetti):
            new_x = x0 + direction_factor * i * float(step)
            transetti.append([float('inf'), new_x])

    #rospy.logwarn("Rette generate (m, q|x0): {}".format(transetti))
    #print("---------------------------------------")

    # Segmenti che delimitano l'area di lavoro
    edges = []
    for i in range(len(w_local)):
        x1, y1 = float(w_local[i][0]), float(w_local[i][1])
        if i < len(w_local) - 1:
            x2, y2 = float(w_local[i+1][0]), float(w_local[i+1][1])
        else:
            x2, y2 = float(w_local[0][0]),  float(w_local[0][1])
        edges.append([[x1, y1], [x2, y2]])

    # Calcolo waypoint dei transetti (prendere anche i waypoint iniziali del bordo all'inizio)
    w_trajectory = []
    w_trajectory.append(w_local[idx0])

    for m_t, q_t in transetti:
        x_list = []
        # Trova tutte le intersezioni tra retta transetto e i segmenti del bordo
        for (p1, p2) in edges:
            ok, xI, yI = intersect_line_with_segment(m_t, q_t, p1, p2)
            if ok:
                add_unique(x_list, xI, yI, 0.0)

        # Ordina le intersezioni lungo la retta per coerenza (per x se non-verticale, per y se verticale)
        if m_t != float('inf'):
            x_list.sort(key=lambda P: P[0])  # ordina per x
        else:
            x_list.sort(key=lambda P: P[1])  # ordina per y

        for pt in x_list:
            w_trajectory.append([pt[0], pt[1], pt[2]])

    #rospy.logwarn("Waypoints transetti grezzi: {}".format(w_trajectory))
    #print("---------------------------------------")

    # Corretto impilamento dei waypoint per i transetti
    if len(w_trajectory) >= 3:
        # Calcola distanze dal primo punto
        d1 = math.sqrt((w_trajectory[1][0]-w_trajectory[0][0])**2 + (w_trajectory[1][1]-w_trajectory[0][1])**2 + (w_trajectory[1][2]-w_trajectory[0][2])**2)
        d2 = math.sqrt((w_trajectory[2][0]-w_trajectory[0][0])**2 + (w_trajectory[2][1]-w_trajectory[0][1])**2 + (w_trajectory[2][2]-w_trajectory[0][2])**2)

        # Copia iniziale
        w_traj_mod = w_trajectory[:]

        # Se il terzo è più vicino, inverti le coppie da indice 1 in poi
        if d2 < d1:
            for i in range(1, len(w_traj_mod) - 1, 2):
                w_traj_mod[i], w_traj_mod[i + 1] = w_traj_mod[i + 1], w_traj_mod[i]

        # Ora genera la serpentina
        new_traj = [w_traj_mod[0]]
        tail = w_traj_mod[1:]

        for b in range(0, len(tail), 2):
            chunk = tail[b:b+2]
            block_idx = b // 2
            if block_idx % 2 == 1 and len(chunk) == 2:
                chunk = chunk[::-1]
            new_traj.extend(chunk)

        w_trajectory = new_traj

    #rospy.logwarn("Waypoints transetti FINAL: {}".format(w_trajectory))
    
    #rospy.logwarn("=== FINE TRANSETTI ===")

    return w_trajectory 

###################################################################
######################### LETTURA DEL JSON ########################
###################################################################

def read_points_from_json(json_file):
    """
    Legge un file JSON con la struttura:
    {
        "areas": [
            {
                "type": "area" (opzionale),
                "points": [
                    {"lat": ..., "lon": ...},
                    ...
                ]
            },
            ...
        ]
    }

    Output:
        (points, tipo)
        dove:
            - points è una lista di [lat, lon, 0]
            - tipo è 'go_to_waypoint' se type non esiste o non è 'area',
              altrimenti 'transetti'
    """
    if not json_file or not os.path.isfile(json_file):
        raise IOError("File non trovato: " + str(json_file))

    try:
        with open(json_file, "r") as f:
            j = json.load(f)
    except Exception as e:
        raise ValueError("Errore nel parsing JSON: " + str(e))

    points = []
    tipo = "go_to_waypoint"

    try:
        for area in j["areas"]:
            # Determina tipo
            if "type" in area and area["type"] == "area":
                tipo = "transetti"

            # Estrai punti
            for p in area["points"]:
                pt = [p["lat"], p["lon"], 0]
                points.append(pt)
    except Exception as e:
        raise ValueError("Errore durante la lettura dei punti: " + str(e))
    
    return points, tipo

###################################################################
########################## PRINT MISSIONE #########################
###################################################################

def plot_mission(A, w_trajectory, WP, latLongVector, mission):
    """
    A, w_trajectory, WP, latLongVector: liste di punti [lat, lon]
    mission: se "transetti" chiude l'area dei WP in blu
    """
    def xy(seq):
        xs = [float(p[1]) for p in seq]  # x=lon
        ys = [float(p[0]) for p in seq]  # y=lat
        return xs, ys

    # Pulisci la figura corrente e ridisegna (modalità interattiva attiva in __init__)
    fig = plt.gcf()
    fig.clf()
    ax = fig.add_subplot(111)

    # --- A in rosso: poligono chiuso ---
    if A is not None and len(A) > 0:
        xA, yA = xy(A)
        ax.plot(xA + [xA[0]], yA + [yA[0]], '-', label='A (area)', linewidth=1.6)
        ax.scatter(xA, yA, s=20, zorder=3)

    # --- WP in blu: chiuso se mission == "transetti" ---
    if WP is not None and len(WP) > 0:
        xW, yW = xy(WP)
        if mission == "transetti":
            ax.plot(xW + [xW[0]], yW + [yW[0]], '-', label='WP (chiuso)', linewidth=1.6)
        else:
            ax.plot(xW, yW, '-', label='WP', linewidth=1.6)
        ax.scatter(xW, yW, s=20, zorder=3)

    # --- w_trajectory in verde: traiettoria (non chiusa) ---
    if w_trajectory is not None and len(w_trajectory) > 0:
        xT, yT = xy(w_trajectory)
        ax.plot(xT, yT, '-', label='w_trajectory', linewidth=1.6)
        ax.scatter(xT, yT, s=20, zorder=3)

    # --- latLongVector in giallo (punto corrente Zeno) ---
    if latLongVector is not None and len(latLongVector) > 0:
        xL, yL = xy(latLongVector)   # atteso [[lat,lon]]
        ax.plot(xL, yL, '-', label='ZENO', linewidth=1.6)
        ax.scatter(xL, yL, s=25, zorder=4, edgecolors='black')

    ax.set_xlabel('Longitudine')
    ax.set_ylabel('Latitudine')
    ax.set_title(u'Safety Area (rosso), Trajectory (verde), WP (blu), Zeno (giallo)')
    ax.grid(True)
    ax.set_aspect('equal', adjustable='box')
    ax.legend()
    fig.tight_layout()

    # Aggiorna senza bloccare
    fig.canvas.draw()
    plt.pause(0.001)
