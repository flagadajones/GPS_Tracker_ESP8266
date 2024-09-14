# GPS_Tracker_ESP8266V1_WEB_BN-220_MSP2

Balise d'identification de drone à distance reglementation Francaise

Basée sur 
https://github.com/dev-fred/GPS_Tracker_ESP8266.git
 

Battit au-dessus de GPS_Tracker_ESP8266V1_WEB_FRSKY, cette version ajoute une sortie MSP2 vers une FC type betaflight/Inav 
ce qui permet d'utiliser le GPS de la balise directement par la FC :
* La latitude, la longitude
* Le nombre de satellites (clignote tant que la position de départ n'est pas définie)
* ...

la sortie MSP2 est cablée sur les pins D7 et D8 de l'ESP8266 D1 mini
le GPS sur les pins D1 et D2

les datas sont envoyées via le message MSP2_SENSOR_GPS
et la structure de données  msp_gps_data_message_t


### Carte
Ce projet utilise un ESP8266 D1 mini.
 