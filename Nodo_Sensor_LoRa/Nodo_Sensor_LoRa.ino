/*******************************************************************************
 * Copyright (c) 2015 Matthijs Kooijman
 *
 * Se autoriza, sin ningún costo, a cualquiera que tenga una copia de este
 * documento y los archivos que lo acompañan a hacer lo que quieran con ellos
 * sin ninguna restricción, incluyendo, pero no limitado a copiar, modificar
 * y redistribuir.
 * NO SE PROVEE NINGUNA GARANTÍA DE NINGÚN TIPO
 *
 * Este ejemplo transmite datos en un canal predefinido y recibe datos cuando
 * no está transmitiendo. Ejecutar este código en dos nodos debería permitirles
 * comunicarse entre sí
 *
 * Modificaciones Martín Faúndez (2018)
 * Se modifica el código anterior, adjuntando la biblioteca DHT de Adafruit y
 * utilizando la función analogRead para poder permitir la medición de humedad,
 * temperatura y viento y enviarlas como datos no encriptados a través de LoRa.
 *******************************************************************************/
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// Definiciones respectivas a la biblioteca DHT
#include "DHT.h"
#define DHTPIN 4
#define DHTTYPE DHT21
DHT dht(DHTPIN, DHTTYPE);

// Creación de la función MAPfloat, la cual funciona exactamente como la función map pero trabajando con valores float
float mapf(float x, float in_min, float in_max, float out_min, float out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#if !defined(DISABLE_INVERT_IQ_ON_RX)
#error Este ejemplo requiere que se habilite la opción DISABLE_INVERT_IQ_ON_RX. Actualiza \
       config.h en la biblioteca lmic para ajustarlo..
#endif

// Cuan a menudo se deberia enviar un paquete. Nótese que este código sobrepasa el límite de 
// limitador de ciclos de trabajo de la biblioteca LMIC, así que cuando cambies algo en esta biblioteca.
// (largo de la carga, frecuencia, factor de ensanchamiento), asegurate de que este valor no deba
// ser incrementado.
// Observa este enlace con información sobre el tiempo en el aire y los ciclos de trabajo:
// https://docs.google.com/spreadsheets/d/1voGAtQAjC1qBmaVuP1ApNKs1ekgUjavHuVQIXyYSvNc
#define TX_INTERVAL 2000

// Configuración de pines de la extensión DRAGINO
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};


// Estos llamados son solamente ocupados en la activacion OTA, así que se
// dejan vacíos aquí ( no los podemos sacar del todo a menos que se configure
// la opción DISABLE_JOIN en config.h, de otra forma el enlazador va a dar alertas y no compilará).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

void onEvent (ev_t ev) {
}

osjob_t txjob;
osjob_t timeoutjob;
static void tx_func (osjob_t* job);

// Transmite la string indicada a la función
void tx(const char *str, osjobcb_t func) {
  os_radio(RADIO_RST); // Stop RX first
  delay(1); // Espera un momento, sin esto la función os_radio mas abajo afirma, aparentemente porque su estado no ha cambiado aun
  LMIC.dataLen = 0;
  while (*str)
    LMIC.frame[LMIC.dataLen++] = *str++;
  LMIC.osjob.func = func;
  os_radio(RADIO_TX);
  Serial.println("TX");
}

// habilita el modo rx cuando recibe un paquete
void rx(osjobcb_t func) {
  LMIC.osjob.func = func;
  LMIC.rxtime = os_getTime(); // RX _now_
  // Habilita RX "continua" (e.j. sin un timeout, aún espera antes de
  // recibir un paquete)
  os_radio(RADIO_RXON);
  Serial.println("RX");
}

static void rxtimeout_func(osjob_t *job) {
  digitalWrite(LED_BUILTIN, LOW); // off
}

static void rx_func (osjob_t* job) {
  // Parpadea una vez para confirmar la recepción y despues mantiene la luz encendida
  digitalWrite(LED_BUILTIN, LOW); // off
  delay(10);
  digitalWrite(LED_BUILTIN, HIGH); // on

  // Timeout RX (e.j. actualizar el estado del led) después de 3 periodos sin RX
  os_setTimedCallback(&timeoutjob, os_getTime() + ms2osticks(3*TX_INTERVAL), rxtimeout_func);

  // Recalendarizar la TX para que no colisione con la TX del otro lado
  os_setTimedCallback(&txjob, os_getTime() + ms2osticks(TX_INTERVAL/2), tx_func);

  Serial.print("Got ");
  Serial.print(LMIC.dataLen);
  Serial.println(" bytes");
  Serial.write(LMIC.frame, LMIC.dataLen);
  Serial.println();

  // Reiniciar RX
  rx(rx_func);
}

static void txdone_func (osjob_t* job) {
  rx(rx_func);
}

// registrar texto en USART y despues enciende el LED
static void tx_func (osjob_t* job) {
  // decir hola
  float humidity, temperature;
  // leer humidity
  humidity = dht.readHumidity();
  // leer temperature
  temperature = dht.readTemperature();

// Función de lectura del sensor de viento. Debido a que el anemómetro Adafruit lee entre 0 y 116.64 km/h, y toma valores entre
// 80 y 818 en la lectura analógica del pin, tomamos este valor, lo insertamos en la función mapf, obteniendo un valor float
  float sensorValue = analogRead(A0);
  float WindSpeed = mapf(sensorValue, 80, 818, 0, 116.64);
  float wind = 0;
  if (WindSpeed > 0){
    wind = WindSpeed;
  delay(1);        // retraso entre lecturas para estabilidad
  }
  else{
    wind = 0;
  }
// Concatenación de los valores de humedad, temperatura y viento
  String str = "";
  str.concat(humidity);
  str.concat(",");      // Inserción de comas para separar
  str.concat(temperature);
  str.concat(",");
  str.concat(wind);
  tx(str.c_str(), txdone_func);
  os_setTimedCallback(job, os_getTime() + ms2osticks(TX_INTERVAL + random(500)), tx_func);
}

void setup() {
  Serial.begin(115200);
  dht.begin();
  Serial.println("Starting");
  #ifdef VCC_ENABLE
  // Para placas Pinoccio Scout
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
  #endif

  pinMode(LED_BUILTIN, OUTPUT);

  // inicializar entorno de ejecucion
  os_init();

  // Configura estos ajustes una vez, y utilizalos tanto en RX como TX

#if defined(CFG_eu868)
  // Utiliza una frecuencia en el g3 que permita el 10% de ciclos de trabajo.
  LMIC.freq = 869525000;
#elif defined(CFG_us915)
  LMIC.freq = 903900000;
#endif

  // Maximum TX power
  LMIC.txpow = 15;
  // Utiliza un spreading factor medio. esto puede ser incrementado a SF12 para
  // un mejor rango, pero el intervalo deberia ser (significantemente)
  // bajado para cumplir con los limites de ciclos de trabajo.
  LMIC.datarate = DR_SF10;
  // Esto configura CR 4/5, BW125 (excepto para DR_SF7B, que utiliza BW250)
  LMIC.rps = updr2rps(LMIC.datarate);

  Serial.println("Started");
  Serial.flush();

  // Configura el trabajo inicial
  os_setCallback(&txjob, tx_func);
}

void loop() {
  // ejecuta los eventos y trabajos calendarizados
  os_runloop_once();
}
