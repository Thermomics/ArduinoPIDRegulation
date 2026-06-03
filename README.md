# ArduinoPIDRegulation


Voici la structure du dossier:

```
.
├── Python-Code
│   ├── check.py
│   └── Visualisation3.py
├── README.md
└── Temperature_Monitor
    └── Temperature_Monitor.ino
```    

pour lancer le code de visualisation, se positionner dans le dossier `Python-Code` et taper:

```
python3 Visualisation3.py
```
    
et charger le fichier `Temperature_Monitor.ino` dans le logiciel arduino IDE
attention il faut installer deux librairies:

* PID_v1
* lib_I2CLCD.h


TODO : si le temps est inférieur à 30s, ne pas mettre de consigne. mesurer l'état stationnaire avant de lancer la consigne, et la commande doit être desactiver.
TODO : lisser la mesure et lisser la consigne 
