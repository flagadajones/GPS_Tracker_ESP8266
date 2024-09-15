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

#define MSP_BAUD_RATE 115200

MSP msp;
msp_gps_data_message_t raw_sensor = { 0 };

SoftwareSerial mspSerial(D7, D8);





void setup() {
  Serial.begin(115200);
  Serial.println();

   //MSP = D7/D8 de ESP8266 D1 mini
  mspSerial.begin(115200);
  msp.begin(mspSerial);
}


void loop() {
 
  //initialisation des données pour envoi MSP
  raw_sensor.instance = 0;       // sensor instance number to support multi-sensor setups
  raw_sensor.gps_week = 0xFFFF;  // GPS week, 0xFFFF if not available
  raw_sensor.ms_tow = millis();
  raw_sensor.satellites_in_view = 10;
 
    raw_sensor.fix_type = MSP_GPS_FIX_3D;
  
  raw_sensor.horizontal_pos_accuracy = 50;  // [cm]
  raw_sensor.vertical_pos_accuracy = 50;    // [cm]
  raw_sensor.horizontal_vel_accuracy = 0;   // [cm/s]
  raw_sensor.hdop = (uint16_t)10;
  raw_sensor.longitude = (int32_t)( 45.667788 * 10000000.0f);
  raw_sensor.latitude = (int32_t)(-0.303567 * 10000000.0f);
  raw_sensor.msl_altitude = (int32_t)(1000 * (100)) / 10;  // cm
  raw_sensor.ned_vel_north = 0;                                                             // cm/s
  raw_sensor.ned_vel_east = 0;
  raw_sensor.ned_vel_down = 0;
  raw_sensor.ground_course = (uint16_t)(10 * 100);  // deg * 100, 0..36000
  raw_sensor.true_yaw = 65535;                                    // deg * 100, values of 0..36000 are valid. 65535 = no data available
  raw_sensor.year = 2024;
  raw_sensor.month = 9;
  raw_sensor.day = 10;
  raw_sensor.hour =11;
  raw_sensor.min = 4;
  raw_sensor.sec = 59;

  //envoi vers la FC
  msp.send(MSP2_SENSOR_GPS, &raw_sensor, sizeof(raw_sensor));


  
}
