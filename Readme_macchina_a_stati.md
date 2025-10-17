# zeno_mission_manager


** Attention: the repository uses marta_msgs! **

To launch the code use:
 *roslaunch zeno_mission_manager zeno_mission_manager.launch*

In the launch files there is a parameter to specify the path of the directory which contains the zeno mission files that can be loaded.
As default value the path is setted as *"/mission_list"*, starting from the home directory.


--------------------------------------------------------------------------------------------------------------------------------------
-------------------------------     INFORMAZIONI RIGUARDANTI LA PIANIFICAZIONE PER IL VEICOLO       ----------------------------------
--------------------------------------------------------------------------------------------------------------------------------------

I custom msg di riferimento necessari per interfacciarsi con il veicolo Zeno in merito alla pianificazione di una missione (o al raggiungimento di un singolo waypoint) sono i seguenti:

Waypoint.msg
    marta_msgs/Position position  --> contiene i campi latitude, longitude e depth del waypoint di riferimento
    float32 speed                 --> velocità in m/s con cui raggiungere il waypoint
    string control_mode           --> modalità di controllo con cui raggiungere il waypoint.* Opzioni sono: ['depth', 'altitude']. 
    float32 altitude              --> valore di riferimento dell'altitudine dal fondale ** 
                                    
                                    * >>>> AI FINI DEL PROGETTO USARE LA MODALITA' "depth" CON PROFONDITA' SETTATA A 0 <<<<
                                        Definisce se il veicolo deve raggiungere il waypoint controllando la profondità o l'altitudine rispetto al fondale. 
                                    
                                    ** Necessaria solo in caso la modalità di controllo sia stata settata come 'altitude'

WaypointList.msg
    Waypoint[] waypoint_list        --> Lista/Array di waypoint che il veicolo raggiungerà in successione


Sono settati alcuni limiti di sicurezza sui valori di questi campi. 
In particolare:
    - la velocità è limitata nel range      [0.1, 1] (m/s)
    - la profondità è limitata nel range    [0, 100] (m)
    - l'altitude è limitata nel range       [2, 100] (m)

Infine è importante notare che un waypoint con coordinate geodetiche (latitudine e longitudine) a distanza > di 1km dalla posizione corrente del veicolo causa il rifiuto della missione da parte del veicolo stesso.


La gestione della missione del veicolo Zeno è effettuata tramite 5 stati:
    IDLE        --> nessuna missione caricata
    READY       --> missione caricata ma non ancora eseguita (può essere sovrascritta)
    PAUSED      --> mission caricata ed eseguita, ma attualmente in pausa
    RUNNING     --> mission caricata ed attualmente in esecuzione
    COMPLETED   --> mission completata


----------------------------------------------------
Funzionamento del gestore di missione del veicolo
----------------------------------------------------

> All'avvio il veicolo si trova in stato IDLE.

> Per caricare una missione è necessario pubblicare una lista di waypoint (WaypointList.msg) nella topic '/zeno/mm/upload_mission' affinché il veicolo passi nello stato READY. 

> Una volta che è il veicolo è nello stato READY è possibile sovrascivere (semplicemente caricando una nuova waypoint list), cancellare o eseguire la missione caricata. 

> Se la missione viene cancellata, il veicolo torna in stato IDLE.

> Se la missione viene eseguita, il veicolo passa allo stato RUNNING.

> Quando il veicolo sta eseguendo una missione, questa può essere messa in pausa. Il veicolo passerà così nello stato PAUSED.

> Una missione in pausa può essere cancellata (veicolo torna IDLE) o fatta ripartire (veicolo torna RUNNING).

> Non si può caricare una nuova missione mentre il veicolo è negli stati RUNNING e PAUSED, bisogna prima fermare la missione in esecuzione e cancellarla per poterne caricare una nuova.

> Una volta che la missione è completata (tutti i waypoint sono stati raggiunti) il veicolo passa in stato COMPLETED.

> Quando il veicolo è in stato COMPLETED si può far ripartire la stessa missione precedentemente eseguita (ripartirà dal primo waypoint), cancellare la missione (veicolo passa in IDLE) o caricarne direttamente una nuova (veicolo passa a READY).

> Per ESEGUIRE una missione bisogna pubblicare uno std_msgs/Empty sulla topic '/zeno/mm/start_mission'

> Per CANCELLARE una missione bisogna pubblicare uno std_msgs/Empty sulla topic '/zeno/mm/delete_mission'

> Per METTERE IN PAUSE una missione bisogna pubblicare uno std_msgs/Empty sulla topic '/zeno/mm/pause_mission'

> Lo stato corrente del veicolo è invece pubblicato sulla topic '/zeno/mm/mission_status' come std_msg/String

