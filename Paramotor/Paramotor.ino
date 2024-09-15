/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
  
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
  
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
/*------------------------------------------------------------------------------
  30/07/2021
  Author: Fred.Dev
  Platforms: ESP8266
  Language: C++/Arduino
  Basé sur :
  https://github.com/khancyr/droneID_FR
  https://github.com/f5soh/balise_esp32/blob/master/droneID_FR.h (version 1 https://discuss.ardupilot.org/t/open-source-french-drone-identification/56904/98 )
  https://github.com/f5soh/balise_esp32
  FrSkySportTelemetry https://www.rcgroups.com/forums/showthread.php?2245978-FrSky-S-Port-telemetry-library-easy-to-use-and-configurable
  https://github.com/d3ngit/djihdfpv_mavlink_to_msp_V2

  *  Envoi les données GPS de la balise au DJI HD FPV system via le protocole MSP et à la telemetrie FRSKY
  *  ESP8266 TX1 to DJI Air unit RX(115200)
------------------------------------------------------------------------------*/

#include "MSP.h"
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
//installer manuellement http://arduiniana.org/libraries/tinygpsplus/
#include <TinyGPS++.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include "index.h"  //Web page file
#include "droneID_FR.h"


#define MSP_BAUD_RATE 115200

MSP msp;
msp_gps_data_message_t raw_sensor = { 0 };

SoftwareSerial mspSerial(D7, D8);

const char prefixe_ssid[] = "BALISE";  // Enter SSID here
// déclaration de la variable qui va contenir le ssid complet = prefixe + MAC adresse
char ssid[32];

/**
  * CHANGEZ l'ID du drone par celui que Alphatango vous a fourni (Trigramme + Modèle + numéro série) !
*/
//*********************** "000000000000000000000000000000"  // 30 caractères
const char drone_id[31] = "000000000000000000000000000000";  // si l'id est inférieur à 30 caractères, le compléter avec des "0" au début

droneIDFR drone_idfr;

// beacon frame definition
static constexpr uint16_t MAX_BEACON_SIZE = 40 + 32 + droneIDFR::FRAME_PAYLOAD_LEN_MAX;  // default beaconPacket size + max ssid size + max drone id frame size

extern "C" {
#include "user_interface.h"
  int wifi_send_pkt_freedom(uint8 *buf, int len, bool sys_seq);
}

// beacon frame definition
uint8_t beaconPacket[MAX_BEACON_SIZE] = {
  /*  0 - 3  */ 0x80, 0x00, 0x00, 0x00,              // Type/Subtype: managment beacon frame
  /*  4 - 9  */ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  // Destination: broadcast
  /* 10 - 15 */ 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,  // Source
  /* 16 - 21 */ 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,  // Source

  // Fixed parameters
  /* 22 - 23 */ 0x00, 0x00,                                      // Fragment & sequence number (will be done by the SDK)
  /* 24 - 31 */ 0x83, 0x51, 0xf7, 0x8f, 0x0f, 0x00, 0x00, 0x00,  // Timestamp
  /* 32 - 33 */ 0xe8, 0x03,                                      // Interval: 0x64, 0x00 => every 100ms - 0xe8, 0x03 => every 1s
  /* 34 - 35 */ 0x21, 0x04,                                      // capabilities Tnformation

  // Tagged parameters

  // SSID parameters
  /* 36 - 38 */ 0x03, 0x01, 0x06,  // DS Parameter set, current channel 6 (= 0x06), // TODO: manually set it
  /* 39 - 40 */ 0x00, 0x20,        // 39-40: SSID parameter set, 0x20:maxlength:content
};

// Ensure the AP SSID is max 31 letters
// 31 lettres maxi selon l'api, 17 caractères de l'adresse mac, reste 15 pour ceux de la chaine du début moins le caractère de fin de chaine ça fait 14, 14+17=31
static_assert((sizeof(prefixe_ssid) / sizeof(*prefixe_ssid)) <= (14 + 1), "Prefix of AP SSID should be less than 14 letters");
// Vérification drone_id max 30
static_assert((sizeof(drone_id) / sizeof(*drone_id)) <= 31, "Drone ID should be less that 30 letters !");  // 30 lettres + null termination

// ========================================================== //

/* Put IP Address details-> smartphone */
IPAddress apIP(192, 168, 1, 10);
DNSServer dnsServer;

ESP8266WebServer server(80);

#define led_pin 2  //internal blue LED
int led;

void flip_Led() {
  //Flip internal LED
  if (led == HIGH) {
    digitalWrite(led_pin, led);
    led = LOW;
  } else {
    digitalWrite(led_pin, led);
    led = HIGH;
  }
}

#define GPS_9600 9600    // Valeur par défaut
#define GPS_57600 57600  // Autre config possible du GPS
#define GPS_RX_PIN 5     // D1 Brancher le fil Tx du GPS
#define GPS_TX_PIN 4     // D2 Brancher le fil Rx du GPS

SoftwareSerial softSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

#define USE_SERIAL Serial

char buff[14][34];
//0$ID;1$UTC;2$SAT;3$HDOP;4$LNG;5$LAT;6$ALT;7$VMAX;8$STATUS;9$TRAME;10$DEP;11$DLNG;12$DLAT;13$TIME

void handleReadValues() {  //pour traiter la requette de mise à jour de la page HTML
  String mes = "";
  for (int i = 0; i <= 14; i++) {
    if (i != 0) mes += ";";
    mes += (String)i + "$" + buff[i];
  }
  server.send(200, "text/plain", mes);
}

void handleNotFound() {
  server.send_P(200, "text/html", webpage);
}
void beginServer() {
  server.begin();
  //connexion captive
  dnsServer.start(53, "*", apIP);
  server.onNotFound(handleNotFound);

  server.on("/", []() {
    server.send_P(200, "text/html", webpage);
  });
  server.on("/readValues", handleReadValues);
  Serial.println("HTTP server started");
}

//testé sur BN220
void SelectChannels() {
  // CFG-GNSS packet GPS + Galileo + Glonas
  byte packet[] = {
    0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x00, 0x20, 0x07, 0x00, 0x08, 0x10, 0x00,
    0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x04,
    0x08, 0x00, 0x01, 0x00, 0x01, 0x01, 0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01,
    0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00,
    0x01, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x01, 0x00, 0x01, 0x01, 0x2E, 0x75
  };
  sendPacket(packet, sizeof(packet));
}

void BaudRate9600() {
  byte packet[] = { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA2, 0xB5 };
  sendPacket(packet, sizeof(packet));
}

void BaudRate57600() {
  byte packet[] = { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0xC9 };
  sendPacket(packet, sizeof(packet));
}

void Rate166() {
  byte packet[] = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xA6, 0x00, 0x01, 0x00, 0x01, 0x00, 0xBC, 0x9E };
  sendPacket(packet, sizeof(packet));
}

void ubloxInit() {
  byte packet[] = {
    //Preprocessor Pedestrian Dynamic Platform Model Option
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00,  // CFG-NAV5 - Set engine settings
    0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,  // Collected by resetting a GPS unit to defaults. Changing mode to Pedistrian and
    0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00,  // capturing the data from the U-Center binary console.
    0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xC2,

    // Enable UBLOX messages
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47,  // set POSLLH MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x03, 0x01, 0x0F, 0x49,  // set STATUS MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4F,  // set SOL MSG rate
    //0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x30, 0x01, 0x3C, 0xA3,           // set SVINFO MSG rate (every cycle - high bandwidth)
    //0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x30, 0x05, 0x40, 0xA7,           // set SVINFO MSG rate (evey 5 cycles - low bandwidth)
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1E, 0x67,  // set VELNED MSG rate
  };
  sendPacket(packet, sizeof(packet));
}

// Send the packet specified to the receiver.
void sendPacket(byte *packet, byte len) {
  for (byte i = 0; i < len; i++) {
    softSerial.write(packet[i]);
  }
}

uint8_t set_home = 1;




void setup() {
  Serial.begin(115200);
  Serial.println();

  for (uint8_t t = 4; t > 0; t--) {
    Serial.printf("[SETUP] BOOT WAIT %d...\r\n", t);
    Serial.flush();
    delay(1000);
  }

  //connection sur le terrain à un smartphone
  // start WiFi
  WiFi.mode(WIFI_AP);
  WiFi.softAP("esp-captive");
  dnsServer.start(53, "*", WiFi.softAPIP());
  //conversion de l'adresse mac:
  String temp = "HBKING_PARA";
  //concat du prefixe et de l'adresse mac
  temp = String(prefixe_ssid) + "_" + temp;
  //transfert dans la variable globale ssid
  temp.toCharArray(ssid, 32);

  // set default AP settings
  WiFi.softAP(ssid, nullptr, 6, false, 4);  // ssid, pwd, channel, hidden, max_cnx
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.setOutputPower(20.5);  // max 20.5dBm
  delay(100);

  Serial.println();
  Serial.println("Started \\o/");
  Serial.print("IP page WEB: ");
  Serial.println(apIP);
  Serial.print("ID Drone: ");
  Serial.println(drone_id);
  WiFi.printDiag(Serial);

  softap_config current_config;
  wifi_softap_get_config(&current_config);
  current_config.beacon_interval = 1000;
  wifi_softap_set_config(&current_config);

  beginServer();  //lancement du server WEB

  //--------------------------------------------- 57600 ->BAUDRATE 9600
  softSerial.begin(GPS_57600);
  delay(100);  // Little delay before flushing.
  softSerial.flush();
  Serial.println("GPS BAUDRATE 9600");
  BaudRate9600();
  delay(100);  // Little delay before flushing.
  softSerial.flush();
  softSerial.begin(GPS_9600);
  delay(100);  // Little delay before flushing.
  softSerial.flush();
  //--------Init provenant de Betaflight
  ubloxInit();
  delay(100);  // Little delay before flushing.
  softSerial.flush();
  //--------------------------------------------- 9600 ->BAUDRATE 57600
  Serial.println("GPS BAUDRATE 57600");
  BaudRate57600();
  delay(100);  // Little delay before flushing.
  softSerial.flush();
  softSerial.begin(GPS_57600);
  //-------Configure GPS ublox
  delay(100);  // Little delay before flushing.
  softSerial.flush();
  //-------Config RATE = 166 ms/6Hz
  Serial.println("Configure GPS RATE = 6 Hz/166 ms");
  Rate166();
  delay(100);  // Little delay before flushing.
  softSerial.flush();
  //--------Config CHANNELS
  Serial.println("Configure GPS CHANNELS = GPS + Galileo + Glonas");
  SelectChannels();
  delay(100);  // Little delay before flushing.
  softSerial.flush();

  drone_idfr.set_drone_id(drone_id);
  snprintf(buff[0], sizeof(buff[0]), "ID:%s", drone_id);
  delay(5000);

  //built in blue LED -> change d'état à chaque envoi de trame
  pinMode(led_pin, OUTPUT);

  //MSP = D7/D8 de ESP8266 D1 mini
  mspSerial.begin(115200);
  msp.begin(mspSerial);
}

unsigned int TRBcounter = 0;
unsigned int nb_sat = 0;
unsigned int SV = 0;
uint64_t beaconSec = 0;
float VMAX = 0.0;
float altitude_ref = 0.0;
float HLng = 0.0;
float HLat = 0.0;
static uint64_t gpsMap = 0;
unsigned long _heures = 0;
unsigned long _minutes = 0;
unsigned long _secondes = 0;
const unsigned int limite_sat = 5;
const float limite_hdop = 2.0;
float _hdop = 0.0;
unsigned int _sat = 0;
float GPS[] = { 0.0, 0.0, 0.0, 0.0 };
uint8_t Y = 0, M = 0, D = 0, H = 0, MN = 0, S = 0;
unsigned int status = 0;  //status télémétrie
/* Status Widget Balise
    [0]  = "NO STAT",
    [1]  = "NO GPS",
    [2]  = "ATT SAT",
    [3]  = "DEPART"
*/

void loop() {
  server.handleClient();
  dnsServer.processNextRequest();

  // Ici on lit les données qui arrivent du GPS et on les passe à la librairie TinyGPS++ pour les traiter

  while (softSerial.available()) gps.encode(softSerial.read());

  //initialisation des données pour envoi MSP
  raw_sensor.instance = 0;       // sensor instance number to support multi-sensor setups
  raw_sensor.gps_week = 0xFFFF;  // GPS week, 0xFFFF if not available
  raw_sensor.ms_tow = gps.time.value();
  raw_sensor.satellites_in_view = gps.satellites.value();
  if (raw_sensor.satellites_in_view >= 2)
    raw_sensor.fix_type = MSP_GPS_FIX_3D;
  else if (raw_sensor.satellites_in_view >= 1)
    raw_sensor.fix_type = MSP_GPS_FIX_2D;
  else
    raw_sensor.fix_type = MSP_GPS_NO_FIX;
  
  raw_sensor.horizontal_pos_accuracy = 50;  // [cm]
  raw_sensor.vertical_pos_accuracy = 50;    // [cm]
  raw_sensor.horizontal_vel_accuracy = 0;   // [cm/s]
  raw_sensor.hdop = (uint16_t)gps.hdop.hdop();
  raw_sensor.longitude = (int32_t)(gps.location.lng() * 10000000.0f);
  raw_sensor.latitude = (int32_t)(gps.location.lat() * 10000000.0f);
  raw_sensor.msl_altitude = (int32_t)(1000 * (gps.altitude.meters() - altitude_ref)) / 10;  // cm
  raw_sensor.ned_vel_north = 0;                                                             // cm/s
  raw_sensor.ned_vel_east = 0;
  raw_sensor.ned_vel_down = 0;
  raw_sensor.ground_course = (uint16_t)(gps.course.deg() * 100);  // deg * 100, 0..36000
  raw_sensor.true_yaw = 65535;                                    // deg * 100, values of 0..36000 are valid. 65535 = no data available
  raw_sensor.year = gps.date.year();
  raw_sensor.month = gps.date.month();
  raw_sensor.day = gps.date.day();
  raw_sensor.hour = gps.time.hour();
  raw_sensor.min = gps.time.minute();
  raw_sensor.sec = gps.time.second();

  //envoi vers la FC
  msp.send(MSP2_SENSOR_GPS, &raw_sensor, sizeof(raw_sensor));


  // On traite le cas où le GPS a un problème
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    //Serial.println("NO GPS");
    strncpy(buff[8], "NO GPS", sizeof(buff[8]));
    status = 1;  //"NO GPS"
    return;
  }

  // On traite le cas si la position GPS n'est pas valide
  if (!gps.location.isValid()) {
    if (millis() - gpsMap > 1000) {
      SV = gps.satellites.value();
      //Serial.print("Waiting... SAT=");  Serial.println(SV);
      snprintf(buff[8], sizeof(buff[8]), "ATT SAT %u", SV);
      status = 2;  //"ATT SAT"
      //Flip internal LED
      flip_Led();
      gpsMap = millis();
    }
    return;
  } else if (gps.location.age() > 3000) {
    //Serial.println("NO SAT");
    strncpy(buff[8], "NO SAT", sizeof(buff[8]));
    return;
  } else if (gps.location.isUpdated() && gps.altitude.isUpdated() && gps.course.isUpdated() && gps.speed.isUpdated()) {
    // On traite le cas où la position GPS est valide.
    // On renseigne le point de démarrage quand la précision est satisfaisante
    _hdop = gps.hdop.hdop();
    if (gps.satellites.value() > nb_sat) {  //+ de satellites
      nb_sat = gps.satellites.value();
      //Serial.print("NEW SAT="); Serial.println(nb_sat);
      strncpy(buff[8], "+SAT", sizeof(buff[8]));
      snprintf(buff[2], sizeof(buff[2]), "SAT:%u", nb_sat);
      snprintf(buff[3], sizeof(buff[3]), "HDOP:%.2f", _hdop);
    }
    if (gps.satellites.value() < nb_sat) {  //- de satellites
      nb_sat = gps.satellites.value();
      //Serial.print("LOST SAT="); Serial.println(nb_sat);
      strncpy(buff[8], "-SAT", sizeof(buff[8]));
      snprintf(buff[2], sizeof(buff[2]), "SAT:%u", nb_sat);
      snprintf(buff[3], sizeof(buff[3]), "HDOP:%.2f", _hdop);
    }
    if (!drone_idfr.has_home_set() && gps.satellites.value() > limite_sat && gps.hdop.hdop() < limite_hdop) {
      //Serial.println("Setting Home Position");
      HLat = gps.location.lat();
      HLng = gps.location.lng();
      altitude_ref = gps.altitude.meters();
      drone_idfr.set_home_position(HLat, HLng, altitude_ref);

      set_home = 0;

      snprintf(buff[11], sizeof(buff[11]), "DLNG:%.4f", HLng);
      snprintf(buff[12], sizeof(buff[12]), "DLAT:%.4f", HLat);
      strncpy(buff[10], "DEPART", sizeof(buff[10]));

      status = 3;  //"PRET"
      D = gps.date.day();
      M = gps.date.month();
      Y = gps.date.year() - 2000;
    }

    // On actualise les données GPS de la librairie d'identification drone.
    GPS[0] = gps.location.lat();
    GPS[1] = gps.location.lng();
    GPS[2] = gps.altitude.meters();
    GPS[3] = gps.speed.mps();
    //drone_idfr.set_ground_speed(gps.speed.mps());
    drone_idfr.set_ground_speed(GPS[3]);
    drone_idfr.set_heading(gps.course.deg());
    //drone_idfr.set_current_position(gps.location.lat(), gps.location.lng(), gps.altitude.meters());
    drone_idfr.set_current_position(GPS[0], GPS[1], (int16_t)GPS[2]);


    //UTC time
    H = gps.time.hour();
    MN = gps.time.minute();
    S = gps.time.second();
  }

  /**
    * On regarde s'il est temps d'envoyer la trame d'identification drone : 
    *  - soit toutes les 3s,
    *  - soit si le drone s'est déplacé de 30m,
    *  - uniquement si la position Home est déjà définie,
    *  - et dans le cas où les données GPS sont nouvelles.
  */
  if (drone_idfr.has_home_set() && drone_idfr.time_to_send()) {
    float time_elapsed = (float(millis() - beaconSec) / 1000);
    beaconSec = millis();

    //On renseigne page WEB
    snprintf(buff[1], sizeof(buff[1]), "UTC:%d:%d:%d", H, MN, S);
    _sat = gps.satellites.value();
    if (_sat < limite_sat) {
      snprintf(buff[2], sizeof(buff[2]), "--SAT:%u", _sat);
    } else {
      snprintf(buff[2], sizeof(buff[2]), "SAT:%u", _sat);
    }
    _hdop = gps.hdop.hdop();
    if (_hdop > limite_hdop) {
      snprintf(buff[3], sizeof(buff[3]), "++HDOP:%.2f", _hdop);
    } else {
      snprintf(buff[3], sizeof(buff[3]), "HDOP:%.2f", _hdop);
    }
    snprintf(buff[4], sizeof(buff[4]), "LNG:%.4f", GPS[1]);
    snprintf(buff[5], sizeof(buff[5]), "LAT:%.4f", GPS[0]);
    snprintf(buff[6], sizeof(buff[6]), "ALT:%.2f", GPS[2] - altitude_ref);

    /*
      Serial.print(time_elapsed,1); Serial.print("s Send beacon: "); Serial.print(drone_idfr.has_pass_distance() ? "Distance" : "Time");
      Serial.print(" with ");  Serial.print(drone_idfr.get_distance_from_last_position_sent()); Serial.print("m Speed="); Serial.println(drone_idfr.get_ground_speed_kmh()); 
    */
    /**
      * On commence par renseigner le ssid du wifi dans la trame
    */
    // write new SSID into beacon frame
    const size_t ssid_size = (sizeof(ssid) / sizeof(*ssid)) - 1;  // remove trailling null termination
    beaconPacket[40] = ssid_size;                                 // set size
    memcpy(&beaconPacket[41], ssid, ssid_size);                   // set ssid
    const uint8_t header_size = 41 + ssid_size;                   //TODO: remove 41 for a marker
    /**
      * On génère la trame wifi avec l'identification
    */
    const uint8_t to_send = drone_idfr.generate_beacon_frame(beaconPacket, header_size);  // override the null termination

    /**
      * On envoie la trame
    */
    wifi_send_pkt_freedom(beaconPacket, to_send, 0);

    //Flip internal LED
    flip_Led();

    //incrementation compteur de trame de balise envoyé
    TRBcounter++;
    snprintf(buff[9], sizeof(buff[9]), "TRAME:%u", TRBcounter);

    _secondes = millis() / 1000;              //convect millisecondes en secondes
    _minutes = _secondes / 60;                //convertir secondes en minutes
    _heures = _minutes / 60;                  //convertir minutes en heures
    _secondes = _secondes - (_minutes * 60);  // soustraire les secondes converties afin d'afficher 59 secondes max
    _minutes = _minutes - (_heures * 60);     //soustraire les minutes converties afin d'afficher 59 minutes max

    snprintf(buff[13], sizeof(buff[13]), " %dmn:%ds", _minutes, _secondes);

    /**
      * On reset la condition d'envoi
    */
    drone_idfr.set_last_send();
  }
}
