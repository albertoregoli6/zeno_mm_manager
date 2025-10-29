Zeno Mission Manager for robot mode

Il pacchetto zeno_mission_manager è per eseguire missioni in modalità "robottino" fornendo i waypoint di interesse tramite un file json. 

Il codice save_json.py permette di collegarsi al server di UNIFI per ricevere il json e salvarlo nella cartella json con il nome niasca.json.  
  
A quel punto il nodo ROS ammirare.py legge il json ed estrae/calcola i waypoint necessari per una missione "go_to_waypoint" o "transetti".  
  
Nella cartella json vi è una sottocartella niasca con le missioni compiute durante la sperimentazione di Niasca del 10/2025 e la sottocartella example in cui sono gli esempi di missioni fornite dai dronisti del Sant'Anna.
