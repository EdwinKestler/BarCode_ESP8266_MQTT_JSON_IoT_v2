//-------- Customise these values-----------
//---------Bluemix IBM Settings-------------
#define ORG "faltbox"
#define DEVICE_TYPE "ESP:8266"
#define DEVICE_ID "ESPBCODE"
#define USER "flatboxadmin"
#define PASS "FBx_admin2012"
//-------- Customise the above values --------

//-------- Customise these values-----------
//---------Blurmix Topics---------------------

const char publishTopic[] = "iot-2/evt/status/fmt/json/barcode";
const char responseTopic[] ="iotdm-1/response/barcode";
const char manageTopic[] = "iotdevice-1/mgmt/manage/barcode";
const char updateTopic[] = "iotdm-1/device/update/barcode";
const char rebootTopic[] = "iotdm-1/mgmt/initiate/device/barcode/reboot";
const char greenTopic[] = "iotdm-1/mgmt/initiate/device/barcode/green";
const char redTopic[] = "iotdm-1/mgmt/initiate/device/barcode/red";


//-----------Variables de Configuracion del Servicio de NTP
//-------- Configuracion de parametros de servicio remots de hora (NTP Servers:)

IPAddress timeServer(129, 6, 15, 29); // time.nist.gov NTP server 129.6.15.29
const char* ntpServerName = "time.nist.gov";
unsigned int localPort = 2390;  // local port to listen for UDP packets
const int timeZone = -6;  // Eastern central Time (USA)

//Variables de Reloj para espera y envio de paquetes de MQTT
unsigned long publishInterval = 60*1000UL; //Variable configurable remotamente sobre el interbalo de publicacion
unsigned long UPDATESENDTIME = 60*1000UL; //Variable que define el tiempo a trancurrir despues de inicializado el reloj para enviar el primer mensake de MQTT en microsegundos (10*1000UL)= 10segundos
unsigned long RetryConnectTIME = 60*1000UL; //Variable que define el tiempo a trancurrir despues de inicializado el reloj para enviar el primer mensake de MQTT en microsegundos (10*1000UL)= 10segundos


//-------- Variables de ERROR EN ENVIO de paquetes de MQTT ANTES DE REINICIO
#define FAILTRESHOLD 150


