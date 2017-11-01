#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
// Librerias de ESP // MQTT/ JSON FORMAT data
#include <ESP8266WiFi.h>                                              //Libreira de ESPCORE ARDUINO
#include <PubSubClient.h>                                             //https://github.com/knolleary/pubsubclient/releases/tag/v2.3
#include <ArduinoJson.h>                                              //https://github.com/bblanchon/ArduinoJson/releases/tag/v5.0.7
//----------------------------------------------------------------------librerias de TIEMPO NTP
#include <TimeLib.h>                                                  //TimeTracking
#include <WiFiUdp.h>                                                  //UDP packet handling for NTP request
//----------------------------------------------------------------------Librerias de manejo de setup de redes 
#include <ESP8266WebServer.h>                                         //Libreira de html para ESP8266
#include <DNSServer.h>                                                //Libreria de DNS para resolucion de Nombres
#include <WiFiManager.h>                                              //https://github.com/tzapu/WiFiManager
//----------------------------------------------------------------------Librerias de Codigo de Lectora RFID
#include <SoftwareSerial.h>                                           //Libreria de SoftwareSerial para recibir data del sensor
#include "settings.h"                                                 //Libreria local que contiene valores configurables de conexion
//----------------------------------------------------------------------Librerias de OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//----------------------------------------------------------------------Definicion de Neopixels
#include "list.h"                                                     //Libreria local que contiene valores configurables de conexion
int S=0;
//PIN in WEMOS = D7
#define PIN 13

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(16, PIN, NEO_GRB + NEO_KHZ800);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

//----------------------------------------------------------------------Poner el Pin de ADC en modo de sensar el voltaje da la bateria
#define OLED_RESET 0
Adafruit_SSD1306 display(OLED_RESET);
#if (SSD1306_LCDHEIGHT != 64)
//#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
//----------------------------------------------------------------------Poner el Pin de ADC en modo de sensar el voltaje da la bateria
ADC_MODE(ADC_VCC);                                                    //Se opne el pin A0 en modo de Lectura interna 1.8V
//----------------------------------------------------------------------Variables de verificacion de fallas de capa de conexion con servicio
int failed, sent, published,ValidCodesread,InValidCodesread;                                          //Variables de conteo de envios 
//----------------------------------------------------------------------Los Pines para la conexion del modulo de RFID TTL son (GPIO12,GPIO14); //Rx, TX Arduino --> Tx, Rx En RDM6300
SoftwareSerial swSer(12, 14, false, 256);                             //Rx, TX Arduino --> Tx, Rx En BARCODE
//----------------------------------------------------------------------Inicio de cliente UDP
WiFiUDP Udp;                                                          //Cliente UDP para WIFI
//----------------------------------------------------------------------Codigo para estblecer el protocolo de tiempo en red NTP
const int NTP_PACKET_SIZE = 48;                                       //NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE];                                   //Buffer to hold incoming & outgoing packets
boolean NTP = false;                                                  //Bandera que establece el estado inicial del valor de NTP
//----------------------------------------------------------------------Variables del servicio de envio de datos MQTT
char server[] = "eospower.flatbox.io";       //EL ORG es la organizacion configurada para el servicio de Bluemix
char clientId[] = "d:" ORG ":" DEVICE_TYPE ":" DEVICE_ID;             //Variable de Identificacion de Cliente para servicio de MQTT Bluemix 
//----------------------------------------------------------------------Declaracion de Variables Globales (procuar que sean las minimas requeridas.)
int FWVERSION = 1;                                                    //Variable configurable remotamente sobre la vesion e firmware
int SleepState = 0;
unsigned long lastPublishMillis, RetryConnectMillis;                                      //Variable para llevar conteo del tiempo desde la ultima publicacion 
String ISO8601;                                                       //Variable para almacenar la marca del timepo (timestamp) de acuerdo al formtao ISO8601
//----------------------------------------------------------------------definir Parametros de Lector de RFID
String response;
String OldTagRead;                                                    //VAriable para guardar la ultima tag leida y evitar lecturas consecutivas
int readerState = 1;
//----------------------------------------------------------------------Variables Para casignacion de pines para OLED


//----------------------------------------------------------------------Variables Propias del CORE ESP8266 Para la administracion del Modulo
String NodeID = String(ESP.getChipId());                              //Variable Global que contiene la identidad del nodo (ChipID) o numero unico
//----------------------------------------------------------------------Funcion remota para administrar las actulizaciones remotas de las variables configurables desde IBMbluemix
void handleUpdate(byte* payload) {                                    //La Funcion recibe lo que obtenga Payload de la Funcion Callback que vigila el Topico de subcripcion (Subscribe TOPIC)
  StaticJsonBuffer<1536> jsonBuffer;                                  //Se establece un Buffer de 1o suficientemente gande para almacenar los menasajes JSON
  JsonObject& root = jsonBuffer.parseObject((char*)payload);          //Se busca la raiz del mensaje Json convirtiendo los Bytes del Payload a Caracteres en el buffer
  if (!root.success()) {                                              //Si no se encuentra el objeto Raiz del Json
    Serial.println(F("ERROR en la Letura del JSON Entrante"));        //Se imprime un mensaje de Error en la lectura del JSON
    return;                                                           //Nos salimos de la funcion
    }                                                                 //se cierra el condicional
  Serial.println(F("handleUpdate payload:"));                         //si se pudo encontrar la raiz del objeto JSON se imprime u mensje
  root.prettyPrintTo(Serial);                                         //y se imprime el mensaje recibido al Serial  
  Serial.println();                                                   //dejamos una linea de pormedio para continuar con los mensajes de debugging
  JsonObject& d = root["d"];                                          //Se define el objeto "d" como  la raiz del mensaje JSON
  JsonArray& fields = d["fields"];                                    //se define el arreglo "fields" del JSON
  for(JsonArray::iterator it=fields.begin();                          //se daclara una rutina para buscar campos dentro del arreglo 
      it!=fields.end();                                               //si no se encuentra lo que se busca se termina la busqueda
      ++it) {                                                         //se busca el siguiente campo
        JsonObject& field = *it;                                      //se asigna lo que tenga el iterador de campos field
        const char* fieldName = field["field"];                       //se crea l avariable nombre de campo
        if (strcmp (fieldName, "metadata") == 0) {                    //Se confirma valida si el campo contiene "metadata"
          JsonObject& fieldValue = field["value"];                    //Se asigna el valor de campo a el objeto de JSON
          if (fieldValue.containsKey("publishInterval")) {            //Si el Valor del campo contiene la LLave "publishInterval"
            publishInterval = fieldValue["publishInterval"];          //asignar ese valor a la variable global "publishInterval"
            Serial.print(F("publishInterval:"));                      //se imprime un mensaje con ka variable que acaba de modificarse remotamente
            Serial.println(publishInterval);                          //se imprime el nuevo valor de la variable actualizada
          }
        }
        if (strcmp (fieldName, "deviceInfo") == 0){                   //Se confirma valida si el campo contiene "deviceInfo"                  
          JsonObject& fieldValue = field["value"];                    //Se asigna el valor de campo a el objeto de JSON
          if (fieldValue.containsKey("fwVersion")) {                  //Si el Valor del campo contiene la LLave "fwVersion"
            FWVERSION = fieldValue["fwVersion"];                      //asignar ese valor a la variable global "FWVERSION"
            Serial.print(F("fwVersion:"));                            //se imprime un mensaje con ka variable que acaba de modificarse remotamente
            Serial.println(FWVERSION);                                //se imprime el nuevo valor de la variable actualizada
          }
        }
      }
}

//----------------------------------------------------------------------Funcion  que cambia el color de las luces
// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

//----------------------------------------------------------------------Funcion de vigilancia sobre mensajeria remota desde el servicion de IBM bluemix
void callback(char* topic, byte* payload, unsigned int payloadLength){//Esta Funcion vigila los mensajes que se reciben por medio de los Topicos de respuesta;
  Serial.print(F("callback invoked for topic: "));                    //Imprimir un mensaje señalando sobre que topico se recibio un mensaje
  Serial.println(topic);                                              //Imprimir el Topico
  
  if (strcmp (rebootTopic, topic) == 0) {                             //verificar si el topico conicide con el Topico rebootTopic[] definido en el archivo settings.h local
    Serial.println(F("Rebooting..."));                                //imprimir mensaje de Aviso sobre reinicio remoto de unidad.
    ESP.restart();                                                    //Emitir comando de reinicio para ESP8266
  }

  if (strcmp (greenTopic, topic) == 0) {                             //verificar si el topico conicide con el Topico rebootTopic[] definido en el archivo settings.h local
    Serial.println(F("green..."));                                //imprimir mensaje de Aviso sobre reinicio remoto de unidad.
    colorWipe(strip.Color(0, 255, 0), 50); // Green
    delay(500);
    colorWipe(strip.Color(0, 0, 0), 50); // off
    readerState = 2;
    //ESP.restart();                                                    //Emitir comando de reinicio para ESP8266
  }

  if (strcmp (redTopic, topic) == 0) {                             //verificar si el topico conicide con el Topico rebootTopic[] definido en el archivo settings.h local
    Serial.println(F("red..."));                                //imprimir mensaje de Aviso sobre reinicio remoto de unidad.
    colorWipe(strip.Color(255, 0, 0), 50); // Red
    delay(500);
    colorWipe(strip.Color(0, 0, 0), 50); // off
    readerState = 2;
    //ESP.restart();                                                    //Emitir comando de reinicio para ESP8266
  }
  
  if (strcmp (updateTopic, topic) == 0) {                             //verificar si el topico conicide con el Topico updateTopic[] definido en el archivo settings.h local
    handleUpdate(payload);                                            //enviar a la funcion handleUpdate el contenido del mensaje para su parseo.
  } 
}

//----------------------------------------------------------------------definicion de Cliente WIFI para ESP8266 y cliente de publicacion y subcripcion
WiFiClient wifiClient;                                                //Se establece el Cliente Wifi
PubSubClient client(server, 1883, callback, wifiClient);              //se establece el Cliente para el servicio MQTT

//----------------------------------------------------------------------Funcion de Conexion a Servicio de MQTT
void mqttConnect() {
 if (!!!client.connected()) {                                         //Verificar si el cliente se encunetra conectado al servicio
  Serial.print(F("Reconnecting MQTT client to "));                    //Si no se encuentra conectado imprimir un mensake de error y de reconexion al servicio
  colorWipe(strip.Color(127, 0, 127), 50); // naranja
  Serial.println(server);
  delay(100);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(30,0);
  display.println("Reconnecting MQTT client to");
  display.println(server);
  display.display(); 
  char charBuf[30];
  String CID (clientId + NodeID); 
  CID.toCharArray(charBuf, 30);
  if (!!!client.connect(clientId, USER, PASS)) {                                //Si no se encuentra conectado al servicio intentar la conexion con las credenciales Clientid, Metodo de autenticacion y el Tokeno password
    Serial.print(F("."));
    delay (100);
    colorWipe(strip.Color(255, 125, 0), 50); // naranja
    colorWipe(strip.Color(0, 0, 0), 50); // off 
    //esperar 500 milisegundos entre cada intento de conexion al servicio de  MQTT
    readerState = 3;
  }
  Serial.println();                                                   //dejar un espacio en la terminal para diferenciar los mensajes.
 }
}
//----------------------------------------------------------------------Funcion encargada de subscribir el nodo a los servicio de administracion remota y de notificar los para metros configurables al mismo
void initManagedDevice() {
  if (client.subscribe(rebootTopic)) {                                //Subscribir el nodo al servicio de mensajeria de reinicio remoto
    Serial.println(F("subscribe to reboot OK"));                      //si se logro la sibscripcion entonces imprimir un mensaje de exito
  }
  else {
    Serial.println(F("subscribe to reboot FAILED"));                  //Si no se logra la subcripcion imprimir un mensaje de error                
  }
  if (client.subscribe(greenTopic)) {                                //Subscribir el nodo al servicio de mensajeria de reinicio remoto
    Serial.println(F("subscribe to green OK"));                      //si se logro la sibscripcion entonces imprimir un mensaje de exito
  }
  else {
    Serial.println(F("subscribe to green FAILED"));                  //Si no se logra la subcripcion imprimir un mensaje de error                
  }
  if (client.subscribe(redTopic)) {                                //Subscribir el nodo al servicio de mensajeria de reinicio remoto
    Serial.println(F("subscribe to red OK"));                      //si se logro la sibscripcion entonces imprimir un mensaje de exito
  }
  else {
    Serial.println(F("subscribe to red FAILED"));                  //Si no se logra la subcripcion imprimir un mensaje de error                
  }
  
  if (client.subscribe("iotdm-1/device/update")) {                    //Subscribir el nodo al servicio de mensajeria de reinicio remoto
    Serial.println(F("subscribe to update OK"));                      //si se logro la sibscripcion entonces imprimir un mensaje de exito
  }
  else {
    Serial.println(F("subscribe to update FAILED"));                  //Si no se logra la subcripcion imprimir un mensaje de error         
  }

  StaticJsonBuffer<1536> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  JsonObject& d = root.createNestedObject("d");
  JsonObject& metadata = d.createNestedObject("metadata");
  metadata["publishInterval"] = publishInterval;
  JsonObject& supports = d.createNestedObject("supports");
  supports["deviceActions"] = true;  
  //JsonObject& deviceInfo = d.createNestedObject("deviceInfo");
  //deviceInfo["fwVersion"] = FWVERSION;
  char buff[1536];
  root.printTo(buff, sizeof(buff));
  Serial.println(F("publishing device metadata:"));
  Serial.println(buff);
  
  if (client.publish(manageTopic, buff)) {
    Serial.println(F("device Publish ok"));
  }else {
    Serial.println(F("device Publish failed:"));
  }
}

//--------  send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:                 
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}


//-------- Funcion para obtener el paquee de TP y procesasr la fecha hora desde el servidor de NTP
time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println(F("Transmit NTP Request"));
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println(F("Receive NTP Response"));
      NTP = true;
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println(F("No NTP Response :-("));
  return 0; // return 0 if unable to get the time
}

//-------- Fucnion para la apertura y conexion de paquetes de UDP para el servicio de  NTP
void udpConnect() {
  Serial.println(F("Starting UDP"));
  Udp.begin(localPort);
  Serial.print(F("Local port: "));
  Serial.println(Udp.localPort());
  Serial.println(F("waiting for sync"));
   setSyncProvider(getNtpTime);
}

//--------  anager function. Configure the wifi connection if not connect put in mode AP--------//
void wifimanager() {
  WiFiManager wifiManager;
  Serial.println(F("empezando"));
  delay(200);
  display.clearDisplay();
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.println("CONFIG WIFI");
  display.display();
  delay(200);
  display.clearDisplay();  
  if (!  wifiManager.autoConnect("flatwifi")) {
    if (!wifiManager.startConfigPortal("flatwifi")) {
      //reset and try again, or maybe put it to deep sleep
      ESP.reset();
      delay(5000);
    }
  }
}

//------------------------------- setup  ---------------------------------------------//
//-------- Funcion Principal de inicializacion de rutina en modulo 
void setup() {
  Serial.begin(9600);// iniciamos el puerto de comunicaiones en pines 0 y 1 para el envio de mensajes de depuracion y error
  Serial.println(F("initializing Barcode WIFI READER Setup"));
  swSer.begin(9600);  //inciamos el puerto serial por software para la lectora RFID (puertos 12 y13) a 9600bps.
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  //initiate neopixels
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  colorWipe(strip.Color(0, 255, 255), 50); // Blue
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.clearDisplay();
  display.display();
  delay(500);
  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print(F("Starting Wifi Mananger"));
  display.display();  
  delay(200);
  display.clearDisplay();   
  //--------  Funcion de Conexion a Wifi
  while (WiFi.status() != WL_CONNECTED) {//conectamos al wifi si no hay la rutina iniciara una pagina web de configuracion en la direccion 192.168.4.1 
    wifimanager();
    delay(1000);
  }
  Serial.print(F("nWiFi connected, IP address: "));
  display.print(F("nWiFi connected, IP address: "));
  delay(200);
  Serial.println(WiFi.localIP());
  Serial.println();//dejamos una linea en blanco en la terminal 
  //una vez contados al Wifi nos aseguramos tener la hora correcta simepre
  Serial.println(F("Connected to WiFi, Sync NTP time")); //mensaje de depuracion para saber que se intentara obtner la hora
  display.println(F("Connected to WiFi, Sync NTP time")); //mensaje de depuracion para saber que se intentara obtner la hora
  display.display();
  delay(200);
  display.clearDisplay();   
  while (NTP == false) {
    colorWipe(strip.Color(125, 0, 255), 50); // magenta
    udpConnect ();//iniciamos la mensajeria de UDP para consultar la hora en el servicio de NTP remoto (el servidor se configura en 
    delay(500);
    colorWipe(strip.Color(0, 0, 0), 50); // Blue
  }
  Serial.println(F("Time Sync, Connecting to mqtt sevrer"));
  display.println(F("Time Sync, Connecting to mqtt sevrer"));
  display.display();
  colorWipe(strip.Color(255, 0, 255), 50); // purple
  delay(200);
  display.clearDisplay();   
  mqttConnect();//Conectamos al servicio de Mqtt con las credenciales provistas en el archivo "settings.h"
  Serial.println(F("Mqtt Connection Done!, sending Device Data"));
  display.println(F("Mqtt Connection Done!, sending Device Data"));
  display.display();
  delay(200);
  display.clearDisplay();   
  initManagedDevice();//inciamos la administracion remota desde Bluemix
  Serial.println(F("Finalizing Setup")); //enviamos un mensaje de depuracion
  display.println(F("Finalizing Setup")); //enviamos un mensaje de depuracion
  display.display();
  delay(100);
  colorWipe(strip.Color(0, 0, 255), 50); // Blue
  display.clearDisplay();   
}

//-------- funcion que procesa como desplegar y transmitir la hora de acuerdo al formato del ISO8601
void ISO8601TimeStampDisplay(){
  // digital clock display of the time
  ISO8601 = String (year(), DEC);
  ISO8601 += "-";
  ISO8601 += month();
  ISO8601 += "-";
  ISO8601 += day();
  ISO8601 +="T";
  ISO8601 += hour();
  ISO8601 += ":";
  ISO8601 += minute();
  ISO8601 += ":";
  ISO8601 += second();
  ISO8601 +="-06:00";
 }

time_t prevDisplay = 0; // Cuando fue actualizada la hora del reloj

//-------- Funcion de verificacion de hora y formato de la misma
void checkTime () {
   if (timeStatus() != timeNotSet) {
    if (now() != prevDisplay) { //update the display only if time has changed
      prevDisplay = now();
      ISO8601TimeStampDisplay();  
    }
  }
}

//-------- publishData function. Publish the data to MQTT server, the payload should not be bigger than 45 characters name field and data field counts. --------//

boolean publishData(String IDModulo, String tagread, String Tstamp) {
  if (OldTagRead != tagread){
    OldTagRead = tagread;    
    StaticJsonBuffer<250> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    JsonObject& d = root.createNestedObject("d");
    JsonObject& data = d.createNestedObject("data");
    data["chipID"] = IDModulo;
    data["tag"] = tagread;
    data["tstamp"] = Tstamp;
    char Mqttdata[250];
    root.printTo(Mqttdata, sizeof(Mqttdata));
    Serial.println(F("publishing device metadata:")); 
    Serial.println(Mqttdata);
    sent ++;
    if (client.publish(publishTopic, Mqttdata)){
    Serial.println(F("Publish OK"));
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(30,10);
    display.println("OK");
    display.display();
    delay(200);
    display.clearDisplay();  
    published ++;
      failed = 0; 
      }else {
        Serial.println(F("Publish FAILED"));
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(30,0);
        display.println("FAILED");
        display.display();
        delay(500);
        display.clearDisplay();    
        failed ++;
        OldTagRead = "";
      }
  }else{
    colorWipe(strip.Color(255, 255, 255), 50); // Blue
    Serial.print("Este es una lectura consecutiva: ");
    //readerState = 2;  
    Serial.println(tagread);
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.println("repetida:");
    display.println(tagread);
    display.display();
    delay(500);
    display.clearDisplay();  
  }
}

//-------- publishData function. Publish the data to MQTT server, the payload should not be bigger than 45 characters name field and data field counts. --------//
void publishManageData (String Sid0, int env, int fail,int VCread, int InVCread){
  float vdd = ESP.getVcc()/1000 ;
  StaticJsonBuffer<1024> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  JsonObject& d = root.createNestedObject("d");
  JsonObject& metadata = d.createNestedObject("metadata");
  metadata["device_name"] = Sid0;
  metadata["Bateria"] = vdd;
  metadata["enviados"] = env;
  metadata["fallidos"] = fail;
  metadata["Validados"] = VCread;
  metadata["NoValidados"] = InVCread;
  char buff[1024];
  root.printTo(buff, sizeof(buff));
  Serial.println("publishing device metadata:");
  Serial.println(buff);
  if (client.publish(manageTopic, buff)) {
    Serial.println("Manage Publish ok");
     published ++;
     failed = 0; 
  }else {
    Serial.print(F("Manage Publish failed:"));
    failed ++;
  }
}

// this function clears the rest of data on the serial, to prevent multiple scans
void clearSerial() {
   while (Serial.read() >= 0) {
    ; // do nothing
  }
  Serial.flush();
  while (swSer.read() >= 0) {
    ; // do nothing
  }
  swSer.flush();
  //asm volatile ("  jmp 0");
}

void ParseTag(){
   boolean found;
  while (swSer.available() > 0){
    delay(10);
    char c = swSer.read();
    if (c == '\r') {break;}
    response += c;
    }    
    if (response.length() >0){
      Serial.println("Codigo de barras:");
      Serial.println(response);
      delay(100);
      checkTime();
      for (int i = 0; i < NUMBER_OF_ELEMENTS; i++){
        String CurrentString = descriptions [i];
        if (CurrentString == response){
          found = true;
          colorWipe(strip.Color(0, 255, 0), 50); // Green
          delay(500);
          colorWipe(strip.Color(0, 0, 0), 50); // off
          readerState = 2;
          descriptions [i] = "111111111111";
          ValidCodesread ++; 
          break;
          }else{
            found = false;
            // Serial.println(i);
            if(i >= NUMBER_OF_ELEMENTS - 1){
              colorWipe(strip.Color(255, 0, 0), 50); // Red
              delay(500);
              colorWipe(strip.Color(0, 0, 0), 50); // off
              readerState = 2;
              InValidCodesread ++;
              break;
            }
          }
      }
      publishData(NodeID, response, ISO8601);
      // clear serial to prevent multiple reads
      response = "";
      clearSerial();
      // reset reading state
    }
}
//-------- Funcion de Ciclo
void loop() {
  DisplayRDY();

  ParseTag();
 
  // VERIFICAMOS CUANTAS VECES NO SE HAN ENVIOADO PAQUETES (ERRORES)
   if (failed >= FAILTRESHOLD){
    failed =0;
    published =0;
    sent=0;    
    ESP.reset();
  }
  
  //si ha pasado el tiempo establecido entonces publicar los datos
  if(millis() - lastPublishMillis > UPDATESENDTIME) {
    String Msg = String ("MSGfailed" + failed); 
    Serial.println(Msg);
    publishManageData(NodeID, published, failed,ValidCodesread,InValidCodesread);
    UPDATESENDTIME = 30*60*1000UL; 
    lastPublishMillis = millis(); //Actulizar la ultima hora de envio
  } 
  
  if (readerState != 3){
     //Serial.println("ON LineMode");
    //verificar que el cliente de Conexion al servicio se encuentre conectado
    if (!client.loop()) {
      mqttConnect();// si el clinete al servicio MQTT se desconecto volver a conectarlo.
    }
  }

   //si ha pasado el tiempo establecido entonces publicar los datos
   
   if (readerState == 3){
     //Serial.println("OFF LineMode");
    if(millis() - RetryConnectMillis > RetryConnectTIME) {
       RetryConnectTIME = 10*60*1000UL;
       RetryConnectMillis = millis(); //Actulizar la ultima hora de envio
       if (!client.loop()) {
        mqttConnect();// si el clinete al servicio MQTT se desconecto volver a conectarlo.
       }       
    }
   colorWipe(strip.Color(255, 125, 255), 50); // orange 
   }
   
   if( readerState != 3){
   colorWipe(strip.Color(0, 0, 255), 50); // Blue
   readerState = 1;
   } 
}

void DisplayRDY (){
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(30,10);
  display.println("Ready to scan!");
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.display();
  delay(200);
  display.clearDisplay();  
}


