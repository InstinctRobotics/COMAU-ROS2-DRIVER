# Sicurezza alla disconnessione: annullamento del movimento in coda

Stato: modifiche C++ compilate e provate contro server finti su 127.0.0.1; modifiche PDL **non compilate e non
provate su un controller** (nessun compilatore PDL fuori dal controller). Prima di usarle sul robot reale
va fatta la procedura in fondo.

## Il problema

I passi MOVEFLY che il driver manda sulla porta 1101 vengono accodati dal controller con `MOVEFLY ... ADVANCE`
(`arm1_handler.pdl`). Se il lato ROS si disattiva o il socket cade, quei passi restano in coda e il robot li
esegue fino all'ultimo, senza nessuno che lo supervisioni.

## Prima / dopo

| Evento | Prima | Dopo |
|---|---|---|
| `on_deactivate` del driver | chiudeva i socket (mandava solo `D`) | manda `X` (CANCEL) sul canale comandi (1105) **prima** di `D` e prima di chiudere; se non c'è stato streaming non manda niente |
| `on_shutdown`, `on_error`, distruttore | niente | stesso annullamento (idempotente: dopo il primo invio riuscito non ne manda altri) |
| invio del cancel fallito | n/a | 3 tentativi a 10 ms, poi `RCLCPP_ERROR` con istruzioni; la disattivazione prosegue |
| socket 1101 che cade mentre il driver fa streaming | `streaming_=false`, il robot finisce la coda | come uno stop: cancel sul canale 1105 (rinviato finché passa), resume solo entro `stop_resume_deviation` (0,01 rad) dal robot, goal del JTC annullato |
| `motion_server.pdl`: socket 1101 caduto (errore di lettura, comando `D`, riavvio) | `ru_restart_server` richiude e riapre, la coda continua | `ru_restart_server` annulla la coda prima di tutto (stessi flag del comando `X`: `_sb_enable_robot := FALSE`, `_sb_cancel_motion := TRUE`, quindi `cancel_motion` di `arm1_handler` = LOCK, CANCEL ALL, UNLOCK, RESUME, svuota i path); secondo annullamento prima di rimettersi in ascolto |
| driver zitto con socket aperto (processo congelato, rete in stallo) | nessuna reazione | heartbeat: se durante lo streaming MOVEFLY non arriva nulla per `KI_HEARTBEAT_MS` **e il braccio si sta ancora muovendo** (`$CRNT_DATA[1].MOVE_STATE > 0`) annulla e riavvia il socket; se il braccio è fermo è una pausa normale e continua ad ascoltare |

Il percorso normale dei comandi (un MOVEFLY ogni ciclo a 100 Hz) non è cambiato. In `write()` c'è solo un
`try_lock` su un mutex (serializza `write()` con l'annullamento di `on_deactivate`; se è occupato salta il
ciclo, non blocca mai il loop di controllo).

## Parametri

- `KI_HEARTBEAT_MS` in `comau_ros2_server/motion_server.pdl`: 1000 ms (0 = heartbeat spento). Il driver manda un
  passo ogni 10 ms; il buco più lungo che produce di proposito è 500 ms (`kMaxStepInterval` con `fly_min_step`
  attivo), 200 ms con `stream_decimation` 20. 1000 ms = 2 volte il buco più lungo: tollera i ritardi normali
  dello scheduler. Attenzione: un falso positivo costa almeno ~10 s di pausa, perché il client TCP riprova a
  connettersi ogni 10 s se il primo tentativo trova il server non ancora in ascolto (`RobotClient::callback`).
- Tentativi del cancel in C++: 3, a 10 ms (`cancelQueuedMotion`).
- Il cancel dell'annullamento usa il canale comandi (1105), non il 1101: se il 1105 è giù l'annullamento dal
  lato C++ non è possibile e resta quello lato PDL alla chiusura del socket 1101.

L'heartbeat serve soprattutto a basso override: la coda si svuota da sola in pochi centinaia di ms alla velocità
piena, ma a `$ARM_OVR` basso gli stessi passi durano 1/override volte di più (al 10%: ~10 volte), e un driver
morto a metà movimento lascerebbe il robot in moto per secondi.

## Quanti passi restano in coda e quanto movimento

Dal codice:

- Lato C++ non c'è coda: un solo MOVEFLY per ciclo (10 ms, 37 byte), scritto subito sul socket.
- `motion_server` a ogni messaggio `F` **sovrascrive** `vj_joint_pos`: vale l'ultimo, i precedenti si perdono.
- `arm1_handler` emette un solo `MOVEFLY ... ADVANCE` per giro di loop (`DELAY 20` + `DELAY 1` nominali; il giro
  reale misurato è ~50-60 ms, vedi `KNOWN_ISSUES.md`, ipotesi 2). Con `ADVANCE` l'istruzione aspetta quando c'è
  già un moto accodato dietro quello in corso: nel simulatore tarato sul reale `ADVANCE_DEPTH = 1`.
- Quindi alla disconnessione sul controller ci sono al massimo: il moto in corso, 1 moto accodato, e 1 obiettivo
  in `vj_joint_pos` non ancora emesso (se ne viene emesso un altro dopo la caduta, anche quello parte).
  Ogni passo copre la traiettoria percorsa in un giro di loop: ~30-60 ms, cioè 40-75 mm a 1,25 m/s.

Misurato sul simulatore (`tests/test_disconnect_sim.py -s`, dopo 1,1-1,5 s di streaming a 100 Hz e chiusura
brusca del socket, TCP a velocità di crociera). Sono numeri del *modello*, non del robot reale:

| Velocità TCP | Moti in planner alla chiusura | Obiettivo avanti al robot | Strada percorsa dopo la chiusura | Tempo al fermo |
|---|---|---|---|---|
| ~0,56 m/s, **senza** annullamento | 1-2 | ~150 mm | ~150 mm (arriva fino all'ultimo obiettivo) | ~0,7 s |
| ~0,56 m/s, **con** annullamento prima della chiusura | 2 | ~150 mm | ~130 mm | ~0,65 s |
| 1,25 m/s (massimo, `fly_lin_velocity`), senza | 2 | ~480 mm | ~495 mm | ~0,8 s |
| 1,25 m/s, con annullamento | 2 | ~490 mm | ~285 mm | ~0,6 s |

Ordine di grandezza: **0,2-0,4 s di moto residuo, cioè velocità × 0,3-0,4 s (fino a ~0,5 m alla velocità massima)**.
L'annullamento non elimina la frenata (v²/2a, fisica del controller): toglie
il tratto fino all'obiettivo che stava davanti al robot e impedisce che la coda riparta. Il vantaggio relativo è
massimo alle alte velocità e quando l'obiettivo è lontano (lag di inseguimento alto).

## Come provarlo sul robot reale

Da fare con il robot in una cella libera, **a bassa velocità**, con la mano sul pulsante di emergenza del TP e
il TP al 100% (è il moltiplicatore finale). La velocità si abbassa da software: `ros2 topic pub --once
/comau/speed_override std_msgs/msg/Int32 "{data: 10}"` (10%). Usare un movimento lungo (30-40 cm) e libero.

0. Carica il nuovo `motion_server.pdl` (`deploy_to_robot.sh`), compilalo dal TP: **se il compilatore segnala
   errori di sintassi, fermati e mandami il messaggio**. Riavvia i programmi PDL come al solito. Compila e
   installa anche il driver C++ (`colcon build --packages-up-to comau_hardware_interface`).
1. Controllo di base: con il driver attivo e nessun movimento, sul TP/schermo del controller non devono
   comparire `READ_TOUT` né riavvii di `[ROBOT_SERVER]` (verifica che `$DV_TOUT` valga solo per `motion_server`,
   vedi sotto). Fai un movimento normale al 10%: deve andare esattamente come prima.
2. Processo ucciso (provato il PDL, il C++ non fa in tempo): a metà movimento `kill -9` del processo ROS che
   ospita il driver. Atteso: il robot frena entro ~1 s; sul log del controller `[MTION_SERVER]`
   `Restarting Arm Server ...` e `Cancelling the motion queued on the controller (ROS side lost)`. Prima della
   modifica proseguiva fino all'ultimo obiettivo.
3. Disattivazione ordinata (prova il C++): a metà movimento
   `ros2 control set_hardware_component_state <nome> inactive`. Atteso: nel log del driver
   `Deactivation: motions queued on the controller cancelled before closing the connections` *prima* di
   `ComauHardwareInterface deactivated.`, e il robot frena subito.
4. Cavo di rete staccato a metà movimento: atteso che il robot finisca la frenata e si fermi; il controller
   si accorge del socket morto solo con il keep-alive TCP (lento): in questo caso conta l'heartbeat (5).
5. Heartbeat: a metà movimento, al 10%, `kill -STOP <pid del processo ROS>` (il socket resta aperto). Atteso:
   dopo ~1 s, se il braccio si muove ancora, `Heartbeat timeout while moving: no MOVEFLY message for 1000 ms`,
   il robot frena. Poi `kill -KILL <pid>`. Se il braccio ha già finito la coda nel secondo di attesa, non c'è
   timeout (è corretto): ripeti con override più basso (5%) o un movimento più lungo.
6. Dopo ognuna delle prove 2-5: riavvia il driver e verifica che un movimento nuovo parta (se il driver rifiuta
   di riprendere con `Not resuming motion after the stop...` mandare un nuovo goal parte dalla posizione reale).

## Punti PDL da verificare sul controller (non compilabili qui)

1. `$DV_TOUT := KI_HEARTBEAT_MS` prima della `READ` e `:= 0` subito dopo: assumo che `$DV_TOUT` sia una variabile
   *di programma* (ogni server la azzera da solo all'avvio) e che il timeout della `READ` su `NETT:` dia
   l'errore 39992 già gestito dal codice (`ERR_TRAP_ON(39990, 39991, 39992)`). Se invece `$DV_TOUT` fosse globale,
   `robot_server` smetterebbe di restare in attesa dei comandi senza fine (timeout 1 s, riavvii continui):
   sintomo della verifica 1. Rimedio immediato: `KI_HEARTBEAT_MS = 0`.
2. `$ERROR` ancora uguale a 39992 dopo `ERR_TRAP_OFF` (il codice esistente fa già affidamento sul valore dopo
   `ERR_TRAP_OFF` in `ru_check_error`).
3. `$CRNT_DATA[KI_ARM_NUM].MOVE_STATE > 0` come "braccio in moto" (stessa espressione di `robot_server.pdl`, riga 252).
4. `_sb_cancel_motion` dichiarata `EXPORTED FROM pdl_tcp_functions NOSAVE` in `motion_server` (copiata da
   `arm1_handler`; per il collegamento serve che `pdl_tcp_functions` sia caricato prima, come già per gli altri).
5. `IF vb_fly_streaming AND (KI_HEARTBEAT_MS > 0) THEN` e `IF vb_fly_streaming AND ($ERROR = 39992) THEN`:
   booleano AND confronto tra parentesi (come `IF (A) OR (B)` esistenti); un confronto tra costante e costante
   potrebbe dare un avviso del compilatore.
6. `CANCEL ALL` mentre `arm1_handler` è bloccato dentro un `MOVEFLY ... ADVANCE` in attesa di posto: se
   l'istruzione, sbloccata, accodasse comunque un ultimo moto dopo il cancel, resterebbe un passo (≤ ~75 mm a
   1,25 m/s). Per questo `ru_restart_server` annulla due volte (dopo 400 ms, prima di rimettersi in ascolto) e
   azzera `_sb_enable_robot` prima del cancel. Non verificabile senza il controller.
7. `WRITE` con la costante `KI_HEARTBEAT_MS` tra gli argomenti.

## Altro da sapere

- `RobotClient::close()` (chiamato dal distruttore) manda **TERMINATE** (`T`): `robot_server` fa `DEACTIVATE`
  di tutti i programmi PDL. Il nuovo cancel del distruttore del driver arriva prima, ma resta un
  comportamento esistente da conoscere (non toccato qui).
- `RobotClient` non è copiabile e il distruttore chiama `detach()` su un thread che `openRobotThread(false)` ha
  già staccato (`std::terminate` in teoria). Non toccato; il test `test_cancel_on_deactivate` lascia
  apposta il driver "perso" (`_exit` in `main`).
