//Bluetooth BIOZ ESP32
//Version 1
//Version 8 of Bluetooth BIOZ Arduino -> MIGRACIÓN A ESP32

#define print_spi_address_and_data 0  
#define MAX3009_enabled 1             
#define eeprom_enabled 1              
#define SPI_strict_write 0            

#include <SPI.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h> 

// --- PINES ESP32 VSPI ---
#define SCK_PIN 18
#define MISO_PIN 19
#define MOSI_PIN 23
const int SELECT_PIN = 5; 

bool MAX3009_SUCCESS = 0;
bool EEPROM_SUCCESS = 0;

Preferences preferences; 

// =================================================================
// 1. ESTRUCTURAS Y VARIABLES GLOBALES (¡Arriba del todo!)
// =================================================================
struct frequency_struct {
  uint16_t MDIV;
  bool NDIV;
  uint8_t KDIV;
  uint8_t DAC_OSR;
  uint8_t ADC_OSR;
};

//Transmission Variables
const int valueBLEBuffer = 3;    
const int numberReadFifo = 400;  
const int checkSumEachNbValues = 100;  
const int minimumNumberLoop = 100;
const int nbValuesToDiscard = 100;      
const int averageSize = 32;

//Variables for init communication
bool connectionEstablished = 0;  

//Variables used for the transmission loop
bool enableSingleTransmission = 0;
bool firstTimeTransmission = 0;
bool waitingPCResponse = 0;
bool gotPCResponse = 0;

//Variables for sending transmission
bool transmissionEnabled = 0;  
bool transmittingValues = 1;   
int valuesSent = 0, valuesDiscarded = 0;
unsigned long totalSum = 0;
bool transmissionAveraged = 0;

//Variables for sending sweep transmission averaged
int nbValuesQ = 0, nbValuesI = 0;
long Q_avg_val = 0, I_avg_val = 0;

//Variables for calibration / sweep
bool calPins = 1;
bool debugMode = 1, save_eeprom = 1, doublecheck_spi = 1;
uint32_t calibResistor = 0;
bool enableCalib = 0;
bool enableSweep = 0, enableAverageTransmission = 0;
uint8_t nbFrequenciesExpected = 0, nbFreqRecieved = 0;

//Other variables
uint32_t I_global = 0, Q_global = 0;
unsigned long timeBefore;
unsigned long valueReadPC = 0;


// =================================================================
// 2. CONFIGURACIÓN Y FUNCIONES BLE (¡Debajo de las variables!)
// =================================================================
#define SERVICE_UUID_COMMAND "0000FFF0-0000-1000-8000-00805F9B34FB"
#define CHAR_UUID_CONNECT_PC "0000FFF1-0000-1000-8000-00805F9B34FB" 
#define CHAR_UUID_COMMAND    "0000FFF2-0000-1000-8000-00805F9B34FB" 
#define CHAR_UUID_FRAME7     "0000FFF3-0000-1000-8000-00805F9B34FB" 

BLEServer* pServer = NULL;
BLECharacteristic* pCommandCharacteristic = NULL;
BLECharacteristic* pFrame7byteCharacteristic = NULL;
bool deviceConnected = false;

void frame7byte_write(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4, uint8_t byte5, uint8_t byte6, uint8_t byte7) {
  if (deviceConnected) {
    uint8_t bufferSend[7] = {byte1, byte2, byte3, byte4, byte5, byte6, byte7};
    pFrame7byteCharacteristic->setValue(bufferSend, sizeof(bufferSend));
    pFrame7byteCharacteristic->notify(); 
  }
}

void sendCommandBLE(uint8_t* bufferSend, size_t length) {
  if (deviceConnected) {
    pCommandCharacteristic->setValue(bufferSend, length);
    pCommandCharacteristic->notify();
  }
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("¡PC/Móvil conectado por BLE!");
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Dispositivo BLE desconectado. Reiniciando anuncio...");
      BLEDevice::startAdvertising(); 
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue();
      
      if (rxValue.length() >= 3) {
        uint8_t rawData[3];
        rawData[0] = rxValue[0];
        rawData[1] = rxValue[1];
        rawData[2] = rxValue[2];

        uint32_t reordered = 0;

        Serial.print("Raw bytes: ");
        Serial.print(rawData[0], HEX);
        Serial.print(" ");
        Serial.print(rawData[1], HEX);
        Serial.print(" ");
        Serial.println(rawData[2], HEX);

        reordered = (rawData[0] << 16) | (rawData[1] << 8) | rawData[2];

        Serial.print("Value reorder: ");
        Serial.println(reordered, HEX);

        if (waitingPCResponse) {
          valueReadPC = reordered;
          gotPCResponse = 1;  
        } else {
          if (rawData[0] != 0x0D && (rawData[0] >> 4) == 0x0 && (rawData[2] >> 4) == 0x1) {
            if (enableCalib) {
              Serial.println("Error. Calibration still happening...");
            } else {
              Serial.println("Enabling transmission");
              initSequence();

              calPins = reordered & 0x000001;
              if (calPins) Serial.println("Use CALx pins (single trans)");
              else Serial.println("Use ELx pins (single trans)");

              struct frequency_struct freqSweep = { 781, 1, 0x1, 0x3, 0x7 };
              if (!setFrequencyReg(freqSweep.MDIV, freqSweep.NDIV, freqSweep.KDIV, freqSweep.DAC_OSR, freqSweep.ADC_OSR)) {
                uint32_t connect_PCValue = 0xD10001;  
                sendCommandBLE((uint8_t*)&connect_PCValue, 3); 
                return;
              }
              uint8_t data_addr;

              SPI_read(0x41, &data_addr);
              SPI_write(0x41, (data_addr & 0xF8) | (calPins << 2) | (1 << 1) | calPins); 
              enableSingleTransmission = 1;
              firstTimeTransmission = 1;
            }
          } else if (rawData[0] == 0x0D) {
            nbFrequenciesExpected = (reordered & 0x0000FF);
            Serial.print("Sweep command recieved with ");
            Serial.print(nbFrequenciesExpected);
            Serial.println(" frequencies");

            if (nbFrequenciesExpected > 0) {
              if (((reordered >> 8) & 0x1) == 0x1) {
                Serial.println("Transmission averaged: yes");
                transmissionAveraged = 1;
              } else {
                Serial.println("Transmission averaged: no");
                transmissionAveraged = 0;
              }

              calPins = (reordered >> 9) & 0x1;
              if (calPins) Serial.println("Use CALx pins");
              else Serial.println("Use ELx pins");

              enableSweep = 1;
              nbFreqRecieved = 0;
              nbValuesQ = 0;
              nbValuesI = 0;
              Q_avg_val = 0;
              I_avg_val = 0;
              initSequence();

              uint8_t data_addr;
              SPI_read(0x41, &data_addr);
              SPI_write(0x41, (data_addr & 0xF8) | (calPins << 2) | (1 << 1) | calPins);

            } else {
              Serial.println("Error: No frequencies were given");
            }
          } else if (reordered == 0x000002) {
            Serial.println("Init command recieved");
            initSequence();
          } else if (((reordered & 0xF00000) >> 20) == 0x5) {
            connectionEstablished = 1;
          } else if (((reordered & 0xF00000) >> 20) == 0xC) {
            const uint8_t type = (reordered & 0x0F0000) >> 16;
            if (type == 0x00) {
              calPins = (reordered >> 2) & 0x000001;
              debugMode = (reordered >> 3) & 0x000001;
              save_eeprom = (reordered >> 4) & 0x000001;
              doublecheck_spi = (reordered >> 5) & 0x000001;
              if (calPins) Serial.println("Use CALx pins");
              else Serial.println("Use ELx pins");
              if (save_eeprom) Serial.println("Save to eeprom");
              else Serial.println("Don't save to eeprom");
            } else if (type == 0x1) {
              calibResistor = 0;
              nbFrequenciesExpected = ((reordered & 0x00FF00) >> 8) & 0xFF;
              Serial.print("Calibration command recieved with ");
              Serial.print(nbFrequenciesExpected);
              Serial.println(" frequencies");
              
              if (eeprom_enabled && save_eeprom) {
                Serial.println("TODO: Guardar nbFrequenciesExpected en Preferences");
              }
              calibResistor = (reordered & 0x0000FF) << 16;
            } else if (type == 0x02) {
              calibResistor = calibResistor | ((reordered & 0x00FF00) << 8) | (reordered & 0x0000FF);
              Serial.print("Calibration resistor: ");
              Serial.println(calibResistor);
              if (eeprom_enabled) {
                Serial.println("TODO: Guardar calibResistor en Preferences");
              }

              Serial.println("Calibration Sequence Initiated");
              enableCalib = 1;
              nbFreqRecieved = 0;
              initSequence();
            } else {
              Serial.print("Unknown type ");
              Serial.print(type);
              Serial.println(" . (tag 0xC for calibration)");
            }
          } else if (((reordered & 0xF00000) >> 20) == 0x3) {
            if (enableCalib) {
              if (nbFreqRecieved < nbFrequenciesExpected) {
                const uint8_t adc = (reordered & 0x0E0000) >> 17,
                              dac = (reordered & 0x018000) >> 15,
                              kdiv = (reordered & 0x007800) >> 11;
                const uint16_t mdiv = reordered & 0x0003FF;
                const bool ndiv = (reordered & 0x000400) >> 10;
                
                struct frequency_struct calibFreqC = { mdiv, ndiv, kdiv, dac, adc };

                uint32_t I_offset = 0, Q_offset = 0, I_rcal_in = 0, Q_rcal_in = 0, I_rcal_quad = 0, Q_rcal_quad = 0;
                uint8_t calibReturn = calibrationAtFrequency(calibFreqC, &I_offset, &Q_offset, &I_rcal_in, &Q_rcal_in, &I_rcal_quad, &Q_rcal_quad);
                sendBackCalib(nbFreqRecieved, calibReturn, I_offset, Q_offset, I_rcal_in, Q_rcal_in, I_rcal_quad, Q_rcal_quad, calibFreqC);
                nbFreqRecieved++;

                if (nbFreqRecieved == nbFrequenciesExpected) {
                  debugMode = 0;
                  doublecheck_spi = 0;
                  Serial.println("Calibration finished.");
                  
                  frame7byte_write(0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0xFF);
                  enableCalib = 0;
                }
              } else {
                Serial.println("Error: More frequencies than expected...");
                uint32_t sendErrorVal = 0xDC0002;
                sendCommandBLE((uint8_t*)&sendErrorVal, 3);
              }
            } else {
              uint32_t connect_PCValue = 0xDC0001;
              sendCommandBLE((uint8_t*)&connect_PCValue, 3);
              Serial.println("Recieved a frequency calibration before proper calibration sequence initialization");
            }
          } else if (((reordered & 0xF00000) >> 20) == 0x4) {
            if (enableSweep) {
              if (nbFreqRecieved < nbFrequenciesExpected) {
                const uint8_t adc = (reordered & 0x0E0000) >> 17,
                              dac = (reordered & 0x018000) >> 15,
                              kdiv = (reordered & 0x007800) >> 11;
                const uint16_t mdiv = reordered & 0x0003FF;
                const bool ndiv = (reordered & 0x000400) >> 10;

                struct frequency_struct freqSweep = { mdiv, ndiv, kdiv, dac, adc };
                
                if (!setFrequencyReg(freqSweep.MDIV, freqSweep.NDIV, freqSweep.KDIV, freqSweep.DAC_OSR, freqSweep.ADC_OSR)) {
                  frame7byte_write(0xFF, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00);
                  enableSweep = 0;
                  return;
                }
                if (!power_on()) {
                  frame7byte_write(0xFF, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00);
                  enableSweep = 0;
                  return;
                }

                Serial.println("Enabling transmission");
                if (transmissionAveraged) {
                  enableAverageTransmission = 1;
                } else {
                  enableSingleTransmission = 1;
                }
                firstTimeTransmission = 1;

              } else {
                Serial.println("Error: More frequencies than expected...");
              }
            } else {
              Serial.println("Recieved a frequency calibration before proper calibration sequence initialization");
            }
          } else {
            Serial.print("Unknown command recieved: ");
            Serial.println(reordered, HEX);
          }
        }
      }
    }
};

void setupBLE() {
  Serial.println("Iniciando servidor BLE...");
  BLEDevice::init("BIOZ_ESP32"); 
  
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pCommandService = pServer->createService(SERVICE_UUID_COMMAND);

  BLECharacteristic *pConnectPcCharacteristic = pCommandService->createCharacteristic(
                                      CHAR_UUID_CONNECT_PC,
                                      BLECharacteristic::PROPERTY_WRITE
                                    );
  pConnectPcCharacteristic->setCallbacks(new MyCallbacks());

  pCommandCharacteristic = pCommandService->createCharacteristic(
                                      CHAR_UUID_COMMAND,
                                      BLECharacteristic::PROPERTY_READ   |
                                      BLECharacteristic::PROPERTY_NOTIFY
                                    );
  pCommandCharacteristic->addDescriptor(new BLE2902());

  pFrame7byteCharacteristic = pCommandService->createCharacteristic(
                                      CHAR_UUID_FRAME7,
                                      BLECharacteristic::PROPERTY_READ   |
                                      BLECharacteristic::PROPERTY_NOTIFY
                                    );
  pFrame7byteCharacteristic->addDescriptor(new BLE2902());

  pCommandService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID_COMMAND);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("BLE activo. Esperando conexión...");
}

// =================================================================
// 3. SETUP Y LOOP
// =================================================================
void setup() {
  Serial.begin(115200);  // initialize serial output
  //while (!Serial) {
  //  ;
  //}
  Serial.println("Initializing ESP32 BIOZ...");

  // 1. INICIALIZACIÓN DE PINES
  pinMode(SELECT_PIN, OUTPUT);
  digitalWrite(SELECT_PIN, HIGH); // Mantenemos el MAX30009 deseleccionado por defecto
  
  // (PIN_VOLTAGE_REDUCER y CS_PIN_EEPROM han sido eliminados)

  // 2. INICIALIZACIÓN DE LA MEMORIA (Sustituto de la EEPROM)
  if (eeprom_enabled) {
    // Abrimos el espacio "bioz" en la memoria Flash (false = modo lectura/escritura)
    preferences.begin("bioz", false); 
    EEPROM_SUCCESS = 1;
    Serial.println("Memoria interna (Preferences) iniciada con exito.");
  } else {
    EEPROM_SUCCESS = 0;
  }

  // 3. INICIALIZACIÓN DEL BUS SPI
  // Le pasamos -1 en el último parámetro para controlar el SELECT_PIN a mano
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, -1); 
  
  // Mantenemos tu velocidad y modo de SPI
  SPI.beginTransaction(SPISettings(450000, MSBFIRST, SPI_MODE0));

  // 4. COMPROBACIÓN DEL MAX30009
  if (MAX3009_enabled) {
    uint8_t partId = 0x00;
    uint8_t count = 0;
    delay(200); // Damos tiempo a que los 1.8V se estabilicen
    
    while (partId != 0x42 && count < 10) {
      partId = readPartId();
      delay(500);
      count++;
    }
    
    if (partId != 0x42) {
      Serial.print("Error: Couldn't read the proper device ID for MAX3009 (42). Read: ");
      Serial.println(partId, HEX);
      MAX3009_SUCCESS = 0;
    } else {
      Serial.println("MAX30009 ID (42) leido correctamente. SPI funcionando!");
      MAX3009_SUCCESS = 1;
      shut_down(); // Asumo que esta función tuya pone el chip a dormir
    }
  }

  if (MAX3009_SUCCESS) {
    setupBLE();
    Serial.println("Bluetooth® device active, waiting for connections...");
  } else {
    Serial.println("Saltando inicio de BLE debido a fallo en hardware.");
  }
}

void loop() {
  serial_functions(); // Tu función original

  // En ESP32 no usamos while(central.connected()), verificamos el estado general:
  if (deviceConnected) {
    
    // =========================================================
    // SECUENCIA DE CONEXIÓN INICIAL
    // =========================================================
    if (connectionEstablished == 1) {
      connectionEstablished = 0; // Reseteamos la bandera

      uint8_t partId = 0x00;
      uint8_t count = 0;
      
      // 1. Chequeo de la Memoria (Preferences)
      if (!EEPROM_SUCCESS && eeprom_enabled) {
        partId = 0x00;
        count = 0;
        delay(300);
        while (partId != 0x42 && count < 10) {
          eeprom_read_byte(0x0000, &partId);
          Serial.print("Read Memoria ID: ");
          Serial.println(partId, HEX);
          delay(500); // Este delay salva al Watchdog automáticamente
          count++;
        }
        if (partId != 0x42) {
          Serial.println("Error: Couldn't read the proper device ID for Memoria (42).");
          EEPROM_SUCCESS = 0;
        } else {
          EEPROM_SUCCESS = 1;
        }
      }

      // 2. Chequeo del sensor MAX30009
      if (!MAX3009_SUCCESS) {
        partId = 0x00;
        count = 0;
        Serial.println("Re-checking MAX3009 chip");
        while (partId != 0x42 && count < 10) {
          partId = readPartId();
          delay(500); 
          count++;
        }
        if (partId != 0x42) {
          Serial.println("Error: Couldn't read the proper device ID for MAX3009 (42).");
          MAX3009_SUCCESS = 0;
        } else {
          MAX3009_SUCCESS = 1;
          shut_down();
        }
      }

      // 3. Envío de datos de Calibración
      if (eeprom_enabled && EEPROM_SUCCESS) {
        uint8_t eeprom_success_byte, eeprom_nb_freq, eeprom_rcal_2, eeprom_rcal_1, eeprom_rcal_0;
        eeprom_read_byte(0x0001, &eeprom_success_byte);

        if (eeprom_success_byte == 0x01) {
          eeprom_read_byte(0x0002, &eeprom_nb_freq);
          Serial.print("Found ");
          Serial.print(eeprom_nb_freq);
          Serial.println(" calibration values.");

          if (eeprom_nb_freq == 255) {
            Serial.print("Memoria success failed. 255 values were indicated: ");
            Serial.println(eeprom_success_byte);
            uint32_t eepromPC = 0x520001;
            // NUEVO ENVÍO BLE ESP32
            pCommandCharacteristic->setValue((uint8_t*)&eepromPC, 4);
            pCommandCharacteristic->notify();

          } else if (eeprom_nb_freq > 0) {
            uint32_t eepromPC = (0x510 << 12) | (EEPROM_SUCCESS << 9) | (MAX3009_SUCCESS << 8) | (eeprom_nb_freq & 0xFF);
            pCommandCharacteristic->setValue((uint8_t*)&eepromPC, 4);
            pCommandCharacteristic->notify();

            uint8_t bufferSend[7];
            eeprom_read_byte(0x0003, &eeprom_rcal_2);
            eeprom_read_byte(0x0004, &eeprom_rcal_1);
            eeprom_read_byte(0x0005, &eeprom_rcal_0);

            Serial.print("Getting resistor value: ");
            Serial.print(eeprom_rcal_2, HEX);
            Serial.print(eeprom_rcal_1, HEX);
            Serial.println(eeprom_rcal_0, HEX);

            bufferSend[0] = eeprom_rcal_0;
            bufferSend[1] = eeprom_rcal_1;
            bufferSend[2] = eeprom_rcal_2;
            bufferSend[3] = 0x00;
            bufferSend[4] = 0x00;
            bufferSend[5] = 0x01;
            bufferSend[6] = 0x00;
            pFrame7byteCharacteristic->setValue(bufferSend, sizeof(bufferSend));
            pFrame7byteCharacteristic->notify();

            // Step 4 - BUCLE DE ENVÍO DE PÁGINAS
            for (int i = 0; i < eeprom_nb_freq; i++) {
              yield(); // <--- EXTREMADAMENTE IMPORTANTE EN ESP32 PARA EL WATCHDOG
              
              uint8_t readBuffer[1 + 31];
              eeprom_read_page((i + 1) << 5, readBuffer, sizeof(readBuffer));

              uint8_t eeprom_freqNum = readBuffer[1],
                      eeprom_adc = (readBuffer[2] >> 5) & 0x07,
                      eeprom_dac = readBuffer[2] & 0x03,
                      eeprom_kdiv = (readBuffer[3] >> 3) & 0x0F,
                      eeprom_ndiv = (readBuffer[3] >> 2) & 0x01;
              uint16_t eeprom_mdiv = (readBuffer[3] & 0x03) << 8 | readBuffer[4];

              bufferSend[0] = eeprom_mdiv & 0xFF;
              bufferSend[1] = ((eeprom_dac & 0x01) << 7) | ((eeprom_kdiv & 0x0F) << 3) | (eeprom_ndiv << 2) | ((eeprom_mdiv & 0x300) >> 8);
              bufferSend[2] = (eeprom_adc << 1) | ((eeprom_dac >> 1) & 0x01);
              bufferSend[3] = 0x00;
              bufferSend[4] = 0x00;
              bufferSend[5] = 0x02;            
              bufferSend[6] = eeprom_freqNum;
              pFrame7byteCharacteristic->setValue(bufferSend, sizeof(bufferSend));
              pFrame7byteCharacteristic->notify();

              uint8_t eeprom_i_offset = ((readBuffer[5] & 0x0F) << 16) | (readBuffer[6] << 8) | readBuffer[7],
                      eeprom_q_offset = ((readBuffer[8] & 0x0F) << 16) | (readBuffer[9] << 8) | readBuffer[10],
                      eeprom_i_rcal_in = ((readBuffer[11] & 0x0F) << 16) | (readBuffer[12] << 8) | readBuffer[13],
                      eeprom_q_rcal_in = ((readBuffer[14] & 0x0F) << 16) | (readBuffer[15] << 8) | readBuffer[16],
                      eeprom_i_rcal_quad = ((readBuffer[17] & 0x0F) << 16) | (readBuffer[18] << 8) | readBuffer[19],
                      eeprom_q_rcal_quad = ((readBuffer[20] & 0x0F) << 16) | (readBuffer[21] << 8) | readBuffer[22];

              // Offset I/Q
              bufferSend[0] = eeprom_i_offset & 0xFF;
              bufferSend[1] = (eeprom_i_offset >> 8) & 0xFF;
              bufferSend[2] = ((eeprom_i_offset >> 16) & 0x0F) | ((eeprom_q_offset & 0x0F) << 4);
              bufferSend[3] = (eeprom_q_offset >> 4) & 0xFF;
              bufferSend[4] = (eeprom_q_offset >> 12) & 0xFF;
              bufferSend[5] = 0x03;
              bufferSend[6] = eeprom_freqNum;
              pFrame7byteCharacteristic->setValue(bufferSend, sizeof(bufferSend));
              pFrame7byteCharacteristic->notify();

              // Rcal in
              bufferSend[0] = eeprom_i_rcal_in & 0xFF;
              bufferSend[1] = (eeprom_i_rcal_in >> 8) & 0xFF;
              bufferSend[2] = ((eeprom_i_rcal_in >> 16) & 0x0F) | ((eeprom_q_rcal_in & 0x0F) << 4);
              bufferSend[3] = (eeprom_q_rcal_in >> 4) & 0xFF;
              bufferSend[4] = (eeprom_q_rcal_in >> 12) & 0xFF;
              bufferSend[5] = 0x04;
              bufferSend[6] = eeprom_freqNum;
              pFrame7byteCharacteristic->setValue(bufferSend, sizeof(bufferSend));
              pFrame7byteCharacteristic->notify();

              // Rcal quad
              bufferSend[0] = eeprom_i_rcal_quad & 0xFF;
              bufferSend[1] = (eeprom_i_rcal_quad >> 8) & 0xFF;
              bufferSend[2] = ((eeprom_i_rcal_quad >> 16) & 0x0F) | ((eeprom_q_rcal_quad & 0x0F) << 4);
              bufferSend[3] = (eeprom_q_rcal_quad >> 4) & 0xFF;
              bufferSend[4] = (eeprom_q_rcal_quad >> 12) & 0xFF;
              bufferSend[5] = 0x05;
              bufferSend[6] = eeprom_freqNum;
              pFrame7byteCharacteristic->setValue(bufferSend, sizeof(bufferSend));
              pFrame7byteCharacteristic->notify();
            }
            Serial.println("Calib init sequence finished.");
            uint32_t finalEepromPC = 0x5C0001;
            pCommandCharacteristic->setValue((uint8_t*)&finalEepromPC, 4);
            pCommandCharacteristic->notify();

          } else {
            Serial.print("Memoria success failed. 0 values were indicated: ");
            Serial.println(eeprom_success_byte);
            uint32_t eepromPC = (0x520 << 12) | (eeprom_enabled << 9) | (MAX3009_SUCCESS << 8) | 0xFF;
            pCommandCharacteristic->setValue((uint8_t*)&eepromPC, 4);
            pCommandCharacteristic->notify();
          }
        } else {
          Serial.print("Memoria success failed. Value: ");
          Serial.println(eeprom_success_byte);
          uint32_t eepromPC = (0x520 << 12) | (eeprom_enabled << 9) | (MAX3009_SUCCESS << 8) | 0xFF;
          pCommandCharacteristic->setValue((uint8_t*)&eepromPC, 4);
          pCommandCharacteristic->notify();
        }

      } else {
        uint32_t eepromPC = (0x500 << 12) | (eeprom_enabled << 9) | (MAX3009_SUCCESS << 8) | 0xFF;
        pCommandCharacteristic->setValue((uint8_t*)&eepromPC, 4);
        pCommandCharacteristic->notify();
        Serial.println("No memoria or Memoria failed check bit. Sending calib value");
      }
    }

    // =========================================================
    // MODOS DE TRANSMISIÓN DE DATOS (BIOZ)
    // =========================================================
    if (enableSingleTransmission) {
      if (firstTimeTransmission) {
        Serial.println("First time transmission");
        totalSum = 0;
        transmissionEnabled = 0;
        valuesSent = 0;
        valuesDiscarded = 0;
        waitingPCResponse = 0;
        gotPCResponse = 0;
      }
      sendTransmission();
    }
    
    if (enableAverageTransmission) {
      if (firstTimeTransmission) {
        Serial.println("First time transmission averaged");
        totalSum = 0;
        transmissionEnabled = 0;
        valuesSent = 0;
        valuesDiscarded = 0;
        waitingPCResponse = 0;
        gotPCResponse = 0;
      }
      sendTransmissionAverage();
    }
  }

  // Dejamos que el ESP32 atienda las tareas de red/Bluetooth en segundo plano
  delay(2); 
}

void serial_functions() {
  if (Serial.available() > 0) {
    int val = Serial.read();
    
    if (val == '0') {
      shut_down();
    }
    if (val == '1') {
      power_on();
    }
    if (val == '2') {
      Serial.println("Init sequence");
      initSequence();
    }
    if (val == '3') {
      struct frequency_struct freqSweep = { 781, 1, 0x1, 0x3, 0x7 };
      if (!setFrequencyReg(freqSweep.MDIV, freqSweep.NDIV, freqSweep.KDIV, freqSweep.DAC_OSR, freqSweep.ADC_OSR)) {
        Serial.println("ERROR");
        return;
      }
    }
    if (val == 'c') {
      //Serial.println("Calibration");
      //calibration();
    }
    if (val == 'f') {
      uint8_t bioz_status;
      SPI_read(0x20, &bioz_status);
      Serial.print("BIOZ Status 0x20: ");
      Serial.println(bioz_status, HEX);

      uint8_t fifo_read_ptr;
      SPI_read(0x09, &fifo_read_ptr);
      Serial.print("FIFO Read Pointer: ");
      Serial.println(fifo_read_ptr, HEX);

      uint8_t msb, lsb, data[3];
      SPI_read(0x0A, &msb);
      SPI_read(0x0B, &lsb);
      uint16_t sampleCount = ((uint16_t)msb << 8) | lsb;

      Serial.print("Samples in FIFO: ");
      Serial.println(sampleCount);

      uint32_t dataFIFO;
      SPI_read_FIFO(&dataFIFO);                        //Gets the data (tag and data) from FIFO
      uint8_t checkData = check_FIFO_data(&dataFIFO);  //0: Invalid, 1: In Phase, 2: Quadrature, 3: Marker
      uint32_t dataF = dataFIFO & 0x0FFFFF;
      Serial.print("Check data: ");
      Serial.print(checkData);
      Serial.print(" Data: ");
      Serial.println(dataF, HEX);
    }
    if (val == 'i') {
      readPartId();
    }
    if (val == 'r') {
      read_all_registers();
    }
    if (val == 's') {
      soft_reset();
    }
  }
}

void SPI_write(uint8_t address, uint8_t data) {
  digitalWrite(SELECT_PIN, LOW);
  uint8_t bufferWrite[3];
  bufferWrite[0] = address;
  bufferWrite[1] = 0x00;
  bufferWrite[2] = data;

  if (print_spi_address_and_data) {
    Serial.print("Address: 0x");
    Serial.print(bufferWrite[0], HEX);
    Serial.print(" Data: 0x");
    Serial.println(bufferWrite[2], HEX);
  }

  SPI.transfer(bufferWrite, sizeof(bufferWrite));
  digitalWrite(SELECT_PIN, HIGH);

  if (doublecheck_spi || SPI_strict_write) {
    // Check the register has the correct value
    uint8_t byte;
    uint8_t count = 0;
    SPI_read(address, &byte);
    while (checkIfValueCorrect(address, data, byte) && count < 10) {
      Serial.print("Trying to write ");
      Serial.print(data, HEX);
      Serial.print(" to address ");
      Serial.print(address, HEX);
      Serial.print(". Obtaining value ");
      Serial.print(byte, HEX);
      Serial.print(". Retrying ");
      Serial.print(count);
      Serial.println("/10");
      
      digitalWrite(SELECT_PIN, LOW);
      SPI.transfer(bufferWrite, sizeof(bufferWrite));
      digitalWrite(SELECT_PIN, HIGH);
      delay(10);
      
      SPI_read(address, &byte);
      count++;
    }
    if (byte != data && count >= 10) {
      Serial.println("ERROR. The MAX3009 chip did not write correctly the result...");
      unsigned long sendErrorVal = 0xD20001;
      // commandCharacteristic.writeValue(sendErrorVal); // <- COMENTADO POR AHORA (BLE Desactivado)
    } 
  }
}

bool checkIfValueCorrect(uint8_t addr, uint8_t data, uint8_t readByte) {
  bool whilestatment = (data != readByte);
  if (whilestatment) {
    switch (addr) {
      case 0x17:
        //0x17 has MDIV which resets to 0x1 => 0x40
        if (readByte == 0x40) whilestatment = 0;
        break;
      case 0x11:
        // System config.
        whilestatment = 0;
        break;
      default:
        // statements
        whilestatment = 1;
        break;
    }
  }
  return whilestatment;
}

void SPI_read(uint8_t addr, uint8_t *bufferRead) {
  digitalWrite(SELECT_PIN, LOW);
  SPI.transfer(addr);                //Send desired address
  SPI.transfer(0x80);                //Read mode
  *bufferRead = SPI.transfer(0x00);  //Get read values
  digitalWrite(SELECT_PIN, HIGH);
  //byte x = SPI.transfer (0); // get response
  //delay(1);
}

void SPI_read_FIFO(uint32_t *data) {
  uint8_t byte1, byte2, byte3;
  digitalWrite(SELECT_PIN, LOW);
  SPI.transfer(0x0C);  //Send desired address
  SPI.transfer(0x80);  //Read mode
  byte1 = SPI.transfer(0x00);
  byte2 = SPI.transfer(0x00);
  byte3 = SPI.transfer(0x00);
  digitalWrite(SELECT_PIN, HIGH);

  *data = byte1 << 16 | byte2 << 8 | byte3;

  //*tag = (byte1 >> 4);
  //*data = ((byte1 & 0x0F)<<16) | (byte2 <<8) | byte3;
}

void initSequence() {
  //Init sequence as stated in https://github.com/analogdevicesinc/MAX32655_MAX30009/blob/main/MAX30009.c
  uint8_t byte;
  SPI_write(0x20, 1 << 2);  //BIOZ_BG_EN
  delay(200);
  SPI_write(0x11, 0x00);  // clear SHDN
  SPI_write(0x17, 0x00);  // clear PLL_EN
  SPI_write(0x1a, 0x00);  // clear REF_CLK_SEL
  SPI_write(0x11, 0x01);  // RESET

  SPI_read(0x00, &byte);

  //SPI_read(0x00, NUM_STATUS_REGS);	// read and clear all status registers
  //SPI_write(0x0d, AFE_FIFO_SIZE-NUM_SAMPLES_PER_INT);	// FIFO_A_FULL; assert A_FULL on NUM_SAMPLES_PER_INT samples


  SPI_write(0x10, 0x00);
  SPI_write(0x12, 0x04);
  SPI_write(0x13, 0x00);
  SPI_write(0x14, 0x00);
  //SPI_write(0x20, 0x74); //TO_VERIFY
  SPI_write(0x21, 0x20);
  SPI_write(0x23, 0x00);
  SPI_write(0x24, 0xF3);  //No analog high pass filter | gain 10v
  SPI_write(0x25, 0x4A);
  SPI_write(0x26, 0x00);
  SPI_write(0x27, 0xFF);
  SPI_write(0x28, 0x02);
  SPI_write(0x42, 0x01);
  SPI_write(0x43, 0xA0);
  SPI_write(0x50, 0x00);
  SPI_write(0x51, 0x00);
  SPI_write(0x58, 0x07);
  SPI_write(0x81, 0x00);

  SPI_write(0x80, 0x80);                                // A_FULL_EN; enable interrupt pin on A_FULL assertion
  SPI_write(0x18, 0xBB);                                // MDIV
  SPI_write(0x19, 0x01);                                // PLL_LOCK_WNDW
  SPI_write(0x1A, (0 << 6) | (1 << 5));                 // REF_CLK_SEL | CLK_FREQ_SEL
  SPI_write(0x17, (1 << 6) | (0 << 5) | (2 << 1) | 0);  // MDIV | NDIV | KDIV | PLL_EN
  SPI_write(0x22, (2 << 4) | (2 << 2));                 //0x28         // BIOZ_VDRV_MAG=177 mVrm | BIOZ_IDRV_RGE: 5.525 kohm -> current = 32 uArms
  //SPI_write(0x25, (3 << 2) | 3);                        // BIOZ_AMP_RGE | BIOZ_AMP_BW
  SPI_write(0x41, (1 << 1));  // MUX_EN, Elx pins are connected (not CALx pins)
  //SPI_write(0x41, (0 << 2) | (1 << 1) | 1); // ! CONNECT_CAL_ONLY | MUX_EN | CAL_EN -> connect CALx pins
  // ensure to enable the MCU's interrupt pin before enabling BIOZ
  SPI_write(0x20, (3 << 6) | (4 << 3) | (1 << 2) | (0 << 1) | 0);  // BIOZ_DAC_OSR | BIOZ_ADC_OSR | BIOZ_BG_EN |! BIOZ_Q_EN | ! BIOZ_I_EN
  delay(200);

  //setMode(0);
}

void setMdiv(int val) {
  /*
    This function is specifically to change the M divider value
    as it spans over two seperate registers
    */
  uint32_t V = val;

  uint8_t a = V >> 8;

  uint8_t b = V & 0xFF;


  changeReg(0x17, a, 7, 2);  //step 3:
  changeReg(0x18, b, 7, 8);  //MDIV
}

void changeReg(uint8_t regAddr, uint8_t val, uint8_t bit1, uint8_t numBits) {
  /*
  This is a function to change specific bits of the byte
  */
  int i = 0;
  uint8_t x1 = 0;
  for (i = 7; i > bit1; i--) {
    x1 = x1 + pow(2, i);
  }
  uint8_t x2 = 0;
  for (i = 0; i < (bit1 - numBits + 1); i++) {
    x2 = x2 + (pow(2, i));
  }
  uint8_t newBits = x1 + x2;
  uint8_t reg1;
  SPI_read(regAddr, &reg1);
  newBits = reg1 & newBits;
  val = val << (bit1 - numBits + 1);
  newBits = newBits + val;
  SPI_write(regAddr, newBits);
}

/*void setMode(int mode) {

  
  //There are many modes available in the MAX30009 data sheet. This function allows you to set the 
  //registers according to each of those modes simply by entering the number of the mode.
  //**** not all modes available so far ****
    
  //using EX4

  if (mode == 0) {               // from BU advice
    changeReg(0x20, 0x3, 7, 2);  // step 1: set BIOZ_DAC_OSR = 256
    changeReg(0x17, 0x9, 4, 4);  // step 2: set KDIV to get PLL_CLK in range --512
    setMdiv(532);
    changeReg(0x17, 1, 5, 1);     //step4: NDIV to 1, meaning 1024
    changeReg(0x20, 0x07, 5, 3);  //step 5: BIOZ_ADC_OSR to 7, meaning 1024


    changeReg(0x25, 1, 6, 1);  //dc restore
    changeReg(0x25, 0, 7, 1);  //bypass Cext
  }

  if (mode == 1) {
    changeReg(0x20, 0x3, 7, 2);   // step 1: set BIOZ_DAC_OSR = 256
    changeReg(0x17, 0xD, 3, 4);   // step 2: set KDIV to get PLL_CLK in range --8192
    changeReg(0x17, 0x04, 7, 2);  //step 3:
    changeReg(0x18, 0x12, 7, 8);  //MDIV to 511
    changeReg(0x17, 1, 5, 1);     //step4: NDIV to 1, meaning 1024
    changeReg(0x20, 0x07, 5, 3);  //step 5: BIOZ_ADC_OSR to 7, meaning 1024
  }

  if (mode == 2) {
    changeReg(0x20, 0x3, 7, 2);   // step 1: set BIOZ_DAC_OSR = 256
    changeReg(0x17, 0xA, 3, 4);   // step 2: set KDIV to get PLL_CLK in range --1024
    setMdiv(799);                 //step 3: MDIV to 799
    changeReg(0x17, 1, 5, 1);     //step4: NDIV to 1, meaning 1024
    changeReg(0x20, 0x04, 5, 3);  //step 5: BIOZ_ADC_OSR to 4, meaning 128
  }

  if (mode == 3) {
    changeReg(0x20, 0x3, 7, 2);  // step 1: set BIOZ_DAC_OSR = 256
    changeReg(0x17, 0x6, 3, 4);  // step 2: set KDIV to get PLL_CLK in range -64
    setMdiv(499);
    changeReg(0x17, 0x0, 5, 1);   //step4: NDIV to 1, meaning 512
    changeReg(0x20, 0x04, 5, 3);  //step 5: BIOZ_ADC_OSR to 4, meaning 128
  }

  if (mode == 4) {
    changeReg(0x20, 0x3, 7, 2);  // step 1: set BIOZ_DAC_OSR = 256
    changeReg(0x17, 0x3, 3, 4);  // step 2: set KDIV to get PLL_CLK in range --8
    setMdiv(624);
    changeReg(0x17, 1, 5, 1);     //step4: NDIV to 1, meaning 1024
    changeReg(0x20, 0x04, 5, 3);  //step 5: BIOZ_ADC_OSR to 4, meaning 128
  }

  if (mode == 5) {
    changeReg(0x20, 0x3, 7, 2);   // step 1: set BIOZ_DAC_OSR = 256
    changeReg(0x17, 0x1, 3, 4);   // step 2: set KDIV to get PLL_CLK in range --8
    changeReg(0x17, 0x02, 7, 2);  //step 3:
    changeReg(0x18, 0x70, 7, 8);  //MDIV to 624
    changeReg(0x17, 1, 5, 1);     //step4: NDIV to 1, meaning 1024
    changeReg(0x20, 0x04, 5, 3);  //step 5: BIOZ_ADC_OSR to 4, meaning 128
  }
  if (mode == 6) {
    changeReg(0x20, 0x3, 7, 2);   // step 1: set BIOZ_DAC_OSR = 256
    changeReg(0x17, 0x0, 3, 4);   // step 2: set KDIV to get PLL_CLK in range --1
    setMdiv(427);                 //step 3: MDIV to 799
    changeReg(0x17, 0, 5, 1);     //step4: NDIV to 0, meaning 512
    changeReg(0x20, 0x04, 5, 3);  //step 5: BIOZ_ADC_OSR to 4, meaning 128
  }
}
*/

/*void initSequence(){
   for (int i = 0; i < sizeof(spi_data_init) / sizeof(spi_data_init[0]); i++) {
    //uint32_t spi_value = (spi_data[i].address << 16) | (0 << 15) | spi_data[i].data; //0 to write, 1 to read
    SPI_write(spi_data_init[i]);
  }
}*/

void shut_down() {
  //Page 24, Verified
  //Disable BioZ
  uint8_t data_addr;
  SPI_read(0x20, &data_addr);         //Get the current parameters, we don t want to override them
  SPI_write(0x20, data_addr & 0xFC);  //Send the bits to disable BIOZ_Q_EN and BIOZ_I_EN

  //Disable PLL
  SPI_read(0x17, &data_addr);         //Get the current parameters
  SPI_write(0x17, data_addr & 0xFE);  //Send the bits to disable PLL_EN

  //Set SHDN to 1 to enter shutdown mode
  SPI_read(0x11, &data_addr);         //Get the current parameters
  SPI_write(0x11, data_addr | 0x02);  //Send the bits to disable SHDN
}

bool power_on() {
  //Page 24, Verified
  uint8_t data_addr;
  //Set SHDN to 0 to enter normal mode
  SPI_read(0x11, &data_addr);         //Get the current parameters
  SPI_write(0x11, data_addr & 0xFD);  //Send the bits to disable SHDN

  //Disable BioZ, if enabled
  SPI_read(0x20, &data_addr);         //Get the current parameters, we don t want to override them
  SPI_write(0x20, data_addr & 0xFC);  //Send the bits to disable BIOZ_Q_EN and BIOZ_I_EN

  //Enable PLL by setting PLL_EN to 1
  SPI_read(0x17, &data_addr);         //Get the current parameters
  SPI_write(0x17, data_addr | 0x01);  //Send the bits to enable PLL_EN

  uint8_t readValue = 0;
  unsigned long timeNow = millis();
  while (readValue == 0 && millis() - timeNow < 100) {
    SPI_read(0x00, &readValue);
    readValue = readValue & 0x0A;
    //Serial.println(readValue);
    delay(10);
  }

  if (readValue == 0) {
    Serial.println("Timeout. Could't read value of FREQ_LOCK");
    return (0);
  } else {
    //Serial.print("Frequency locked: ");
    //Serial.println(readValue);
  }

  //Enable BioZ I and Q
  SPI_read(0x20, &data_addr);         //Get the current parameters, we don t want to override them
  SPI_write(0x20, data_addr | 0x03);  //Send the bits to enable BIOZ_Q_EN and BIOZ_I_EN
  delay(2);

  delay(10);
  //Flush FIFO
  uint8_t dataFlush;
  SPI_read(0x0E, &dataFlush);         //Get the current parameters
  SPI_write(0x0E, dataFlush | 0x10);  //Send the bits
  return (1);
}

void soft_reset() {
  Serial.println("Reset");
  uint8_t data_addr;
  //BIOZ_BG_EN = 1
  SPI_read(0x20, &data_addr);             //Get the current parameters
  SPI_write(0x20, data_addr | (1 << 2));  //Send the bits

  //Set SHDN to 0 to enter normal mode
  SPI_read(0x11, &data_addr);         //Get the current parameters
  SPI_write(0x11, data_addr & 0xFD);  //Send the bits

  //Set REF_CLK_SEL = 0
  SPI_read(0x1A, &data_addr);         //Get the current parameters
  SPI_write(0x1A, data_addr & 0xBF);  //Send the bits

  //Set PLL_EN = 0
  SPI_read(0x17, &data_addr);         //Get the current parameters
  SPI_write(0x17, data_addr & 0xFE);  //Send the bits

  //Delay 1 ms
  /*unsigned long previousMicros = micros();
  while (micros() - previousMicros <= 2000)
    ;  //Wait 1000 us = 1 ms
  */
  delay(5);
  //Set RESET = 0
  SPI_read(0x11, &data_addr);         //Get the current parameters
  SPI_write(0x11, data_addr | 0x01);  //Send the bits

  //Set PLL_EN = 1
  /*SPI_read(0x17, &data_addr);         //Get the current parameters
  SPI_write(0x17, data_addr | 0x01);  //Send the bits*/
}

void read_all_registers() {
  uint8_t byte1;
  uint8_t addr_to_read[36] = { 0x00, 0x01, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x10, 0x11, 0x12, 0x13, 0x14, 0x17, 0x18, 0x19, 0x1A, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x41, 0x42, 0x43, 0x44, 0x50, 0x51, 0x52, 0x58, 0xFF };
  Serial.println("Reading all registers\nAddress,Value");
  for (uint8_t i = 0; i < sizeof(addr_to_read); i++) {
    SPI_read(addr_to_read[i], &byte1);
    char strBuf[50];
    sprintf(strBuf, "%02X,%02X", addr_to_read[i], byte1);
    Serial.println(strBuf);
    if (debugMode) {
      uint8_t bufferSend[3];
      bufferSend[0] = byte1;
      bufferSend[1] = addr_to_read[i];
      bufferSend[2] = 0xA0;
      //commandCharacteristic.writeValue(bufferSend, sizeof(bufferSend));
    }
  }
}

/*void calibration() {
  //struct frequency_struct {
  //uint16_t MDIV;
  //  bool NDIV;
  //  uint8_t KDIV;
  //  uint8_t DAC_OSR;
  //  uint8_t ADC_OSR;
  //};

  frequency_struct calibFreq[] = {
    { 625, 1, 0x4, 0x3, 0x5 },  //Set drive frequency to 5 kHz
    { 625, 1, 0x2, 0x3, 0x5 },  //Set drive frequency to 20 kHz
    { 781, 1, 0x1, 0x3, 0x5 },  //Set drive frequency to 50 kHz
    { 781, 1, 0x0, 0x3, 0x5 },  //Set drive frequency to 99.968 kHz
    { 586, 1, 0x0, 0x2, 0x5 },  //Set drive frequency to 150,016 kHz
    { 781, 1, 0x0, 0x2, 0x5 },  //Set drive frequency to 199,936 kHz
    { 488, 0, 0x0, 0x1, 0x6 },  //Set drive frequency to 249.856 kHz
    { 586, 1, 0x0, 0x1, 0x5 },  //Set drive frequency to 300,032 kHz
  };
  initSequence();
  uint8_t nbCalib = sizeof(calibFreq) / sizeof(calibFreq[0]);

  Serial.println(nbCalib);
  for (uint8_t i = 0; i < nbCalib; i++) {
    uint32_t I_offset = 0, Q_offset = 0, I_rcal_in = 0, Q_rcal_in = 0, I_rcal_quad = 0, Q_rcal_quad = 0;
    uint8_t calibReturn = calibrationAtFrequency(calibFreq[i], &I_offset, &Q_offset, &I_rcal_in, &Q_rcal_in, &I_rcal_quad, &Q_rcal_quad);
    sendBackCalib(i, calibReturn, I_offset, Q_offset, I_rcal_in, Q_rcal_in, I_rcal_quad, Q_rcal_quad, calibFreq[i]);
    //i = nbCalib;  //For dev purposes
  }
  //calibrationAtFrequency(uint16_t MDIV, bool NDIV, uint8_t KDIV, uint8_t DAC_OSR, uint8_t ADC_OSR)
}*/

void sendBackCalib(uint8_t freqNum, uint8_t calibReturn, uint32_t I_offset, uint32_t Q_offset, uint32_t I_rcal_in, uint32_t Q_rcal_in, uint32_t I_rcal_quad, uint32_t Q_rcal_quad, frequency_struct calibReg) {
  if (calibReturn == 1) {
    uint8_t bufferSend[7];
    bufferSend[0] = I_offset & 0xFF;
    bufferSend[1] = (I_offset >> 8) & 0xFF;
    bufferSend[2] = ((I_offset >> 16) & 0x0F) | ((Q_offset & 0x0F) << 4);
    bufferSend[3] = (Q_offset >> 4) & 0xFF;
    bufferSend[4] = (Q_offset >> 12) & 0xFF;
    bufferSend[5] = 0x00;     //Type I/Q offset, no error
    bufferSend[6] = freqNum;  //Freq number
    
    if (deviceConnected) {
      pFrame7byteCharacteristic->setValue(bufferSend, sizeof(bufferSend));
      pFrame7byteCharacteristic->notify();
    }

    bufferSend[0] = I_rcal_in & 0xFF;
    bufferSend[1] = (I_rcal_in >> 8) & 0xFF;
    bufferSend[2] = ((I_rcal_in >> 16) & 0x0F) | ((Q_rcal_in & 0x0F) << 4);
    bufferSend[3] = (Q_rcal_in >> 4) & 0xFF;
    bufferSend[4] = (Q_rcal_in >> 12) & 0xFF;
    bufferSend[5] = 0x01;     //Type I/Q Rcal in, no error
    bufferSend[6] = freqNum;  //Freq number
    
    if (deviceConnected) {
      pFrame7byteCharacteristic->setValue(bufferSend, sizeof(bufferSend));
      pFrame7byteCharacteristic->notify();
    }

    bufferSend[0] = I_rcal_quad & 0xFF;
    bufferSend[1] = (I_rcal_quad >> 8) & 0xFF;
    bufferSend[2] = ((I_rcal_quad >> 16) & 0x0F) | ((Q_rcal_quad & 0x0F) << 4);
    bufferSend[3] = (Q_rcal_quad >> 4) & 0xFF;
    bufferSend[4] = (Q_rcal_quad >> 12) & 0xFF;
    bufferSend[5] = 0x02;     //Type I/Q Rcal quad, no error
    bufferSend[6] = freqNum;  //Freq number
    
    if (deviceConnected) {
      pFrame7byteCharacteristic->setValue(bufferSend, sizeof(bufferSend));
      pFrame7byteCharacteristic->notify();
    }

    if (eeprom_enabled && save_eeprom) {
      Serial.println("Saving to eeprom");
      uint8_t pageWrite[24] = {
        0x42,     //Integrity bit
        0x16,     //22 bytes to follow
        freqNum,  //Frequency number (0 - 126)
        (calibReg.ADC_OSR << 5) | (calibReg.DAC_OSR),
        (calibReg.KDIV << 3) | (calibReg.NDIV << 2) | ((calibReg.MDIV & 0x300) >> 8),
        calibReg.MDIV & 0x0FFFF,
        (I_offset >> 16) & 0x0F,
        (I_offset >> 8) & 0xFF,
        I_offset & 0xFF,
        (Q_offset >> 16) & 0x0F,
        (Q_offset >> 8) & 0xFF,
        Q_offset & 0xFF,
        (I_rcal_in >> 16) & 0x0F,
        (I_rcal_in >> 8) & 0xFF,
        I_rcal_in & 0xFF,
        (Q_rcal_in >> 16) & 0x0F,
        (Q_rcal_in >> 8) & 0xFF,
        Q_rcal_in & 0xFF,
        (I_rcal_quad >> 16) & 0x0F,
        (I_rcal_quad >> 8) & 0xFF,
        I_rcal_quad & 0xFF,
        (Q_rcal_quad >> 16) & 0x0F,
        (Q_rcal_quad >> 8) & 0xFF,
        Q_rcal_quad & 0xFF,
      };
      // Write at address multiple of 32
      //starting by 32 for freqNum=0
      eeprom_write_page((freqNum + 1) << 5, pageWrite, 24);
      Serial.println("TODO: Guardar página en Preferences");
    } else Serial.println("EEPROM is disabled. Calibration values were not stored.");

  } else {
    debugMode = 0;
    doublecheck_spi = 0;
    Serial.print("ABORT. Seems like there was an error... error_code = ");
    Serial.println(calibReturn);
    if (eeprom_enabled) eeprom_write_byte(0x0001, calibReturn);
    Serial.println("TODO: Guardar página en Preferences");
    uint8_t bufferSend[7];
    bufferSend[0] = 0x00;
    bufferSend[1] = 0x00;
    bufferSend[2] = 0x00;
    bufferSend[3] = 0x00;
    bufferSend[4] = 0x00;
    bufferSend[5] = (calibReturn & 0x0F) << 4;  //Return error type
    bufferSend[6] = freqNum;                    //Freq number
    
    if (deviceConnected) {
      pFrame7byteCharacteristic->setValue(bufferSend, sizeof(bufferSend));
      pFrame7byteCharacteristic->notify();
    }

  }
}

uint8_t calibrationAtFrequency(frequency_struct calibReg, uint32_t *I_offset_pt, uint32_t *Q_offset_pt, uint32_t *I_rcal_in_pt, uint32_t *Q_rcal_in_pt, uint32_t *I_rcal_quad_pt, uint32_t *Q_rcal_quad_pt) {
  //Get the calibration values for a certain frequency
  //Error table: 0: Undefined error, 1: no error, 2: Power on error,
  //3: settling error offset, 4: settling error rcal_in, 5: settling error rcal_quad
  //Page 46
  uint8_t data_addr;

  if (!setFrequencyReg(calibReg.MDIV, calibReg.NDIV, calibReg.KDIV, calibReg.DAC_OSR, calibReg.ADC_OSR)) return (0);

  //BioZ MUX configuration to connect CALx pins
  //SPI_read(0x41, &data_addr);         //Get the current parameters
  //SPI_write(0x41, data_addr | 0x07);  //CONNECT_CAL_ONLY=1, MUX_EN=1, CAL_EN=1;

  //BioZ MUX configuration to calibrate using the ELX pins
  Serial.print("Calibration pin used : ");
  Serial.println(calPins);

  SPI_read(0x41, &data_addr);
  SPI_write(0x41, (data_addr & 0xF8) | (calPins << 2) | (1 << 1) | calPins);  //CalPins = 1 => Use cal pin
                                                                              //CalPin = 0 => Use ELX pin

  SPI_write(0x42, 0x1);


  //Measure I and Q offset

  //NOPE: Set the stimulus current magnitude to the minimum 16nARMS by setting BIOZ_VDRV_MAG[5:4](0x22) and BIOZ_IDRV_RGE[3:2](0x22) to 0x0.
  //Set stimulus to
  SPI_read(0x22, &data_addr);         //Get the current parameters
  SPI_write(0x22, data_addr & 0xC3);  //Send the bits
  //SPI_write(0x22, (2 << 4) | (2 << 2));  //0x28, (from initialization, we want current = 32 uarms)

  //Enable BIOZ_DRV_RESET[5](0x25) to apply a short circuit across the load
  SPI_read(0x25, &data_addr);             //Get the current parameters
  SPI_write(0x25, data_addr | (1 << 5));  //Send the bits
  delay(10);

  if (!power_on()) return (2);
  Serial.println("Calibration initialization sequence OK. Getting I and Q offset");

  // --- EL PARCHE QUE FALTABA: ENCENDER EL ADC ANTES DE ESPERAR DATOS ---
  uint8_t temp_reg20;
  SPI_read(0x20, &temp_reg20);
  SPI_write(0x20, temp_reg20 | 0x03); // Enciende BIOZ_I_EN y BIOZ_Q_EN
  // ---------------------------------------------------------------------

  //Wait for settling time and return I offset and Q offset
  if (!settleAndReturn(I_offset_pt, Q_offset_pt)) {
    Serial.println("Error during settle and return offset");
    //Disable measurement
    SPI_read(0x20, &data_addr);
    SPI_write(0x20, data_addr & 0xFC);
    //Disable BIOZ_DRV_RESET[5] (0x25)
    SPI_read(0x25, &data_addr);
    SPI_write(0x25, data_addr & 0xDF);
    //Connect resistors to ELx pins
    SPI_read(0x41, &data_addr);
    SPI_write(0x41, (data_addr & 0xF8) | (1 << 1));
    shut_down();
    return (3);
  }

  //Serial.println("Settle and return OK");

  //Disable measurement
  SPI_read(0x20, &data_addr);
  SPI_write(0x20, data_addr & 0xFC);

  //Serial.println("Disable measurment OK");

  //Measure the calibration resistor with both channels set to in phase

  //Set the stimulus current to the desired value by adjusting BIOZ_VDRV_MAG[5:4] and BIOZ_IDRV_RGE[3:2] (0x22)
  SPI_write(0x22, (2 << 4) | (2 << 2));  //0x28, (from initialization, we want current = 32 uarms)

  //Serial.println("Stimulus current set OK");

  //Disable BIOZ_DRV_RESET[5] (0x25)
  SPI_read(0x25, &data_addr);
  SPI_write(0x25, data_addr & 0xDF);

  //Serial.println("Disable BIOZ_DRV_RESET[5] OK");

  //Set BIOZ_Q_CLK_PHASE[3](0x28) to 1, which shifts the Q-channel's demodulation clock to in-phase
  SPI_read(0x28, &data_addr);
  SPI_write(0x28, (data_addr & 0xF3) | (1 << 3));

  //Serial.println("Set BIOZ_Q_CLK_PHASE[3](0x28) to 1 OK");

  //Set BIOZ_I_EN[0] (0x20) and BIOZ_Q_EN[1] (0x20) to 1 to enable measurement
  SPI_read(0x20, &data_addr);
  SPI_write(0x20, data_addr | 0x03);

  //Serial.println("Set BIOZ_I_EN[0] (0x20) and BIOZ_Q_EN[1] (0x20) to 1 OK");


  Serial.println("I and Q offset value successful. Getting rcal in");
  //Record data until impedance signal is settled, and then record the average impedance I_rcal_in and Q_rcal_in
  if (!settleAndReturn(I_rcal_in_pt, Q_rcal_in_pt)) {
    Serial.println("Error during settle and return rcal in");
    //Disable measurement
    SPI_read(0x20, &data_addr);
    SPI_write(0x20, data_addr & 0xFC);
    //Connect resistors to ELx pins
    SPI_read(0x41, &data_addr);
    SPI_write(0x41, (data_addr & 0xF8) | (1 << 1));
    shut_down();
    return (4);
  }

  //Disable measurement
  SPI_read(0x20, &data_addr);
  SPI_write(0x20, data_addr & 0xFC);

  //Set BIOZ_Q_CLK_PHASE[3](0x28) to 0. Set BIOZ_I_CLK_PHASE[2](0x28) to 1, which shifts the I-channel's demodulation clock to quadrature phase.
  SPI_read(0x28, &data_addr);
  SPI_write(0x28, (data_addr & 0xF3) | (1 << 2));

  //Set BIOZ_I_EN[0] (0x20) and BIOZ_Q_EN[1] (0x20) to 1 to enable measurement
  SPI_read(0x20, &data_addr);
  SPI_write(0x20, data_addr | 0x03);

  Serial.println("I and Q rcal in value successful. Getting rcal quad");

  if (!settleAndReturn(I_rcal_quad_pt, Q_rcal_quad_pt)) {
    Serial.println("Error during settle and return rcal quad");
    SPI_read(0x20, &data_addr);
    SPI_write(0x20, data_addr & 0xFC);
    //Connect resistors to ELx pins
    SPI_read(0x41, &data_addr);
    SPI_write(0x41, (data_addr & 0xF8) | (1 << 1));
    shut_down();
    return (5);
  }

  SPI_read(0x20, &data_addr);
  SPI_write(0x20, data_addr & 0xFC);

  Serial.println("Calibration END ");
  shut_down();
  //Serial.print(loopNum);
  //Serial.print("/");
  //Serial.print(maxLoop);

  SPI_read(0x41, &data_addr);
  SPI_write(0x41, (data_addr & 0xF8) | (1 << 1));  //CONNECT resistor to ELx pin (not CAL)

  return (1);
}

bool setFrequencyReg(uint16_t MDIV, bool NDIV, uint8_t KDIV, uint8_t DAC_OSR, uint8_t ADC_OSR) {
  /*
  Sets the frequency of the PLL
  */
  //Check if the parameters given from the user are valid
  float PLL_CLK = MDIV * 32.768e3;
  float bioz_adc_clk = PLL_CLK / (NDIV ? 1024 : 512);
  float bioz_synth_clk = PLL_CLK / (1 << KDIV);
  if (PLL_CLK < 14e6 || PLL_CLK > 28e6) {
    Serial.print("ABORT. The user frequency is ");
    Serial.print(PLL_CLK / 1e6);
    Serial.println(" MHz. The frequency PLL_CLK must be between 14MHz and 28 MHz.");
    return (0);
  }
  if (bioz_adc_clk < 16.0e3 || bioz_adc_clk > 36.375e3) {
    Serial.print("ABORT. The user frequency is ");
    Serial.print(bioz_adc_clk / 1e3);
    Serial.println(" kHz. The frequency BIOZ_ADC_CLK must be between 16kHz and 36.375 kHz.");
    return (0);
  }
  if (bioz_synth_clk < 4096 || bioz_synth_clk > 28e6) {
    Serial.print("ABORT. The user frequency is ");
    Serial.print(bioz_synth_clk / 1e6);
    Serial.println(" MHz. The frequency BIOZ_SYNTH_CLK must be between 4096 Hz and 28 MHz.");
    return (0);
  }

  Serial.print("User configures a frequency of: ");
  Serial.print((bioz_synth_clk / (1 << (5 + DAC_OSR))) / 1e3);
  Serial.println(" kHz");
  Serial.print("Frequencies: PLL = ");
  Serial.print(PLL_CLK / 1e6);
  Serial.print(" MHz, BIOZ_ADC_CLK = ");
  Serial.print(bioz_adc_clk / 1e3);
  Serial.print(" KHz, BIOZ_SYNTH_CLK = ");
  Serial.print(bioz_synth_clk / 1e6);
  Serial.println(" MHz.");
  //return (0);  // For dev purposes

  uint8_t byte;
  SPI_read(0x1A, &byte);
  SPI_write(0x1A, (byte & 0xBF) | 0x20);  //Internal oscilator of 32.768

  //Setting the PLL frequency and KDIV
  MDIV--;  //Because datasheet says it multiplies by MDIV + 1
  SPI_read(0x17, &byte);
  SPI_write(0x17, (byte & 0x01) | ((MDIV & 0x0300) >> 2) | NDIV << 5 | ((KDIV & 0x0F) << 1));  //Sets the 2 MSB of MDIV and the NDIV and KDIV wanted
  SPI_write(0x18, MDIV & 0x00FF);                                                              //Sets the rest of MDIV

  //Setting the BioZ DAC OSR and BIOZ ADC OSR
  SPI_read(0x20, &byte);
  SPI_write(0x20, (byte & 0x07) | ((DAC_OSR & 0x03) << 6) | (ADC_OSR & 0x07) << 3);  //Set the ADC_OSR and DAC_OSR

  return (1);


  //KDIV; DAC OSR; N DIV; ADC OSR; MDIV
}

bool settleAndReturn(uint32_t *I_average, uint32_t *Q_average) {

  // 1. CAMBIAMOS A int32_t (con signo) PARA ACEPTAR NEGATIVOS
  int32_t sumQ = 0, sumI = 0;
  uint16_t countQ = 0, countI = 0, loopNum = 0;
  int32_t data_I[averageSize], data_Q[averageSize];
  
  // Timeout de seguridad de 4 segundos
  unsigned long startTime = millis();

  while ((countQ < averageSize || countI < averageSize)) {
    
    // Si se atasca, imprimimos exactamente DÓNDE se ha atascado
    if (millis() - startTime > 4000) {
      Serial.println("\n[ERROR] TIMEOUT! El chip no entrega suficientes datos validos.");
      Serial.print("I recibidos: "); Serial.print(countI);
      Serial.print(" | Q recibidos: "); Serial.print(countQ);
      Serial.print(" | Ciclos validos (loopNum): "); Serial.println(loopNum);
      return false; 
    }

    uint32_t dataFIFO;
    uint16_t sampleCount = 0;
    uint8_t msb, lsb;
    
    // Leemos cuántos datos hay
    SPI_read(0x0A, &msb);
    SPI_read(0x0B, &lsb);
    sampleCount = ((uint16_t)msb << 8) | lsb;

    if (sampleCount > 0) {
      SPI_read_FIFO(&dataFIFO);                        
      uint8_t checkData = check_FIFO_data(&dataFIFO);  
      
      // --- EL FIX DE LOS NÚMEROS NEGATIVOS ---
      int32_t data = dataFIFO & 0x0FFFFF; // Nos quedamos con los 20 bits
      if (data & 0x80000) {               // Si el bit 19 (el de signo) es un 1...
        data = data | 0xFFF00000;         // ...rellenamos con 1s a la izquierda para hacerlo negativo en 32 bits
      }
      // ---------------------------------------

      // Solo procesar y sumar si el dato es 100% válido (Tag 1 o 2)
      if (checkData == 1 || checkData == 2) {
        
        if (loopNum >= minimumNumberLoop+nbValuesToDiscard) {
          if (checkData == 1 && countQ < averageSize) {
            data_Q[countQ] = data; // Guardamos el número (positivo o negativo)
            countQ++;
          } else if (checkData == 2 && countI < averageSize) {
            data_I[countI] = data;
            countI++;
          }
        }
        loopNum++; 
      }
    }
    
    delay(1); 
  }
  
  if (countI == averageSize && countQ == averageSize) {
    Serial.print("Measure finished. Compiling results. loopNum = ");
    Serial.println(loopNum);
    
    // Apagamos la medición
    uint8_t data_addr;
    SPI_read(0x20, &data_addr);
    SPI_write(0x20, data_addr & 0xFC);

    // Calculamos la media (internamente el Arduino la hará bien con los negativos)
    for (int i = 0; i < averageSize; i++) {
      sumQ += data_Q[i];
      sumI += data_I[i];
    }
    
    int32_t real_avg_I = sumI / averageSize;
    int32_t real_avg_Q = sumQ / averageSize;

    // --- EL FIX PARA ENGAÑAR A LA WEB ---
    // Como la web no entiende negativos, le aplicamos el valor absoluto (abs).
    real_avg_I = abs(real_avg_I);
    real_avg_Q = abs(real_avg_Q);
    // ------------------------------------

    // Lo devolvemos enmascarado a 20 bits para que el Bluetooth lo envíe al PC
    *I_average = (uint32_t)(real_avg_I) & 0x0FFFFF;
    *Q_average = (uint32_t)(real_avg_Q) & 0x0FFFFF;
    
    return true;
  } else {
    return false;
  }
}

uint8_t check_FIFO_data(uint32_t *data) {
  /*Checks the FIFO data type
    * 0: Invalid
    * 1: In Phase
    * 2: Quadrature
    * 3: Marker
  */
  uint8_t tag = *data >> 20;
  if (tag == 0b0001) return 1;
  if (tag == 0b0010) return 2;
  if (tag == 0b1111) {
    if (*data == 0xFFFFFE) return 3;
    if (*data == 0xFFFFFF) return 0;
  }
  return 0;
}

uint8_t readPartId() {
  uint8_t byte1;
  SPI_read(0xFF, &byte1);
  Serial.print("Read part ID: ");
  Serial.println(byte1, HEX);
  return byte1;
}

/*uint8_t readFIFOByte(){
  uint8_t byte1;
  SPI_read(0x0C, &byte1);
  Serial.print("FIFO: ");
  Serial.println(byte1, HEX);
  return byte1;
}*/

// --- FUNCIONES PUENTE EEPROM -> PREFERENCES ---

void eeprom_write_byte(uint16_t addr, uint8_t val) {
  if (!eeprom_enabled) return;
  // Convertimos la dirección numérica en una clave de texto (ej: "a1", "a256")
  String key = "a" + String(addr);
  preferences.putUChar(key.c_str(), val);
  Serial.printf("Memoria: Guardado byte %02X en clave %s\n", val, key.c_str());
}

void eeprom_read_byte(uint16_t addr, uint8_t *val) {
  if (!eeprom_enabled) {
    *val = 0;
    return;
  }
  String key = "a" + String(addr);
  // El segundo parámetro (0) es el valor por defecto si no encuentra nada
  *val = preferences.getUChar(key.c_str(), 0);
}

void eeprom_write_page(uint16_t addr, uint8_t* buf, size_t len) {
  if (!eeprom_enabled) return;
  String key = "p" + String(addr);
  preferences.putBytes(key.c_str(), buf, len);
  Serial.printf("Memoria: Guardada página de %d bytes en clave %s\n", len, key.c_str());
}

void eeprom_read_page(uint16_t addr, uint8_t* buf, size_t len) {
  if (!eeprom_enabled) return;
  String key = "p" + String(addr);
  preferences.getBytes(key.c_str(), buf, len);
}

void sendTransmission() {
  if (firstTimeTransmission) {
    uint8_t data_addr;
    if (!power_on()) {  
      //Power on failed. Sending error to PC
      unsigned long connect_PCValue = 0xD10001;
      sendCommandBLE((uint8_t*)&connect_PCValue, 4); // Adaptado a ESP32
      enableSingleTransmission = 0;
      return;
    } else firstTimeTransmission = 0;
  }

  if (valuesSent < numberReadFifo && enableSingleTransmission || waitingPCResponse) {
    uint16_t sampleCount = 0;
    if (!waitingPCResponse) {
      uint8_t msb, lsb, data[3];
      SPI_read(0x0A, &msb);
      SPI_read(0x0B, &lsb);
      sampleCount = ((uint16_t)msb << 8) | lsb;

      if (sampleCount > checkSumEachNbValues + 5) {
        Serial.print("Samples in FIFO: ");
        Serial.println(sampleCount);
        if (sampleCount == 0xFFFF) {
          uint8_t dataFlush;
          SPI_read(0x0E, &dataFlush);         
          SPI_write(0x0E, dataFlush | 0x10);
        }
      }
    }

    if (waitingPCResponse || ((sampleCount > checkSumEachNbValues || transmissionEnabled))) {
      unsigned long connect_PCValue;
      if (!waitingPCResponse) {
        transmissionEnabled = sampleCount > 0;
        uint32_t dataFIFO;
        SPI_read_FIFO(&dataFIFO);

        if (valuesDiscarded < nbValuesToDiscard) {
          Serial.println("Discarded");
          valuesDiscarded++;
        } else {
          valuesSent++;
          Serial.print(dataFIFO, HEX);
          Serial.print(", TAG: ");
          Serial.print(dataFIFO >> 20, HEX);
          Serial.print(", DATA: ");
          Serial.print(dataFIFO & 0x0FFFFF, HEX);

          if ((dataFIFO & 0xFFFFFF) != 0xFFFFFF) {
            totalSum += dataFIFO & 0x0FFFFF;
            Serial.print(", CHECKSUM: ");
            Serial.print(totalSum, HEX);
            Serial.print(", Timestamp: ");
            Serial.print(millis());
            Serial.print(", Samples in FIFO: ");
            Serial.println(sampleCount);
            
            connect_PCValue = (unsigned long)dataFIFO;
            sendCommandBLE((uint8_t*)&connect_PCValue, 4); // Adaptado a ESP32
          } else {
            Serial.println(", ERR TAG");
          }
        }
      }

      if ((valuesSent % checkSumEachNbValues == 0 && valuesDiscarded > nbValuesToDiscard && 0) || valuesSent == numberReadFifo || waitingPCResponse) {
        if (!waitingPCResponse) {
          Serial.println("Checking with PC");
          connect_PCValue = 0x0FFFFF;
          waitingPCResponse = 1;
          sendCommandBLE((uint8_t*)&connect_PCValue, 4); // Adaptado a ESP32
          timeBefore = millis();
        }
        
        unsigned long timeDiff = millis() - timeBefore;
        if (waitingPCResponse && !gotPCResponse && timeDiff < 5000) {
          return;
        }
        waitingPCResponse = 0;

        if (!gotPCResponse) {
          Serial.println("Timeout error: Waiting for PC");
          enableSingleTransmission = 0;
        } else {
          Serial.print("PC: ");
          Serial.print(valueReadPC, HEX);
          Serial.print(" | Arduino: ");
          Serial.println(totalSum & 0xFFFFFF, HEX);

          if (valueReadPC != (totalSum & 0xFFFFFF)) {
            Serial.println("Checksum mismatch");
            enableSingleTransmission = 0;
          } else {
            Serial.println("Checksum OK");
          }
          gotPCResponse = 0;
        }
      }
    }
  }

  if ((!waitingPCResponse && valuesSent == numberReadFifo) || !enableSingleTransmission) {
    Serial.println("Transmission end");
    shut_down();
    enableSingleTransmission = 0;
    Serial.print("Checksum: ");
    Serial.println(totalSum, HEX);

    if (enableSweep) {
      unsigned long valueEnd = 0xE00D00 | (nbFreqRecieved & 0xFF);
      Serial.print("End of transmission for this sweep frequency number ");
      Serial.print(nbFreqRecieved);
      Serial.print(". Sending EOT message ");
      Serial.println(valueEnd, HEX);
      
      sendCommandBLE((uint8_t*)&valueEnd, 4); // Adaptado a ESP32
      
      nbFreqRecieved++;
      if (nbFreqRecieved == nbFrequenciesExpected) {
        enableSweep = 0;
      }
    } else {
      doublecheck_spi = 0;
      Serial.println("Sending normal transmission END message");
      unsigned long valueEnd = 0xE00001;
      sendCommandBLE((uint8_t*)&valueEnd, 4); // Adaptado a ESP32
    }
  }
}

void sendTransmissionAverage() {
  if (firstTimeTransmission) {
    uint8_t data_addr;
    if (!power_on()) {  
      unsigned long connect_PCValue = 0xD10001;
      sendCommandBLE((uint8_t*)&connect_PCValue, 4); // Adaptado a ESP32
      enableAverageTransmission = 0;
      return;
    } else firstTimeTransmission = 0;
    
    nbValuesQ = 0;
    nbValuesI = 0;
    Q_avg_val = 0;
    I_avg_val = 0;
  }
  
  if (valuesSent < numberReadFifo && enableAverageTransmission) {
    uint16_t sampleCount = 0;
    uint8_t msb, lsb, data[3];
    SPI_read(0x0A, &msb);
    SPI_read(0x0B, &lsb);
    sampleCount = ((uint16_t)msb << 8) | lsb;

    if (sampleCount > checkSumEachNbValues + 5) {
      Serial.print("Samples in FIFO: ");
      Serial.println(sampleCount);
      if (sampleCount == 0xFFFF) {
        uint8_t dataFlush;
        SPI_read(0x0E, &dataFlush);         
        SPI_write(0x0E, dataFlush | 0x10);
      }
    }
    
    if ((sampleCount > checkSumEachNbValues || transmissionEnabled)) {
      unsigned long connect_PCValue;
      transmissionEnabled = sampleCount > 0;
      uint32_t dataFIFO;
      SPI_read_FIFO(&dataFIFO);

      if (valuesDiscarded < nbValuesToDiscard) {
        valuesDiscarded++;
      } else {
        if (dataFIFO >> 20 == 0x1) {
          if (nbValuesI < numberReadFifo) {
            nbValuesI++;
            int32_t current_I = dataFIFO & 0x0FFFFF;
            if (current_I & 0x80000) { current_I |= 0xFFF00000; } 

            if (nbValuesI == 1) {
              I_avg_val = current_I;
            } else {
              I_avg_val = ((int32_t)I_avg_val * (nbValuesI - 1) / nbValuesI) + (current_I / nbValuesI);
            }
          }
        }
        
        if (dataFIFO >> 20 == 0x2) {
          if (nbValuesQ < numberReadFifo) {
            nbValuesQ++;
            int32_t current_Q = dataFIFO & 0x0FFFFF;
            if (current_Q & 0x80000) { current_Q |= 0xFFF00000; } 

            if (nbValuesQ == 1) {
              Q_avg_val = current_Q;
            } else {
              Q_avg_val = ((int32_t)Q_avg_val * (nbValuesQ - 1) / nbValuesQ) + (current_Q / nbValuesQ);
            }
          }
        }

        if (nbValuesQ == numberReadFifo && nbValuesI == numberReadFifo) {
          valuesSent = numberReadFifo;
          uint32_t I_avg_val_20 = (uint32_t)I_avg_val & 0x0FFFFF;
          uint32_t Q_avg_val_20 = (uint32_t)Q_avg_val & 0x0FFFFF;

          Serial.print("Average I raw: ");
          Serial.println((int32_t)I_avg_val);
          Serial.print("Average Q raw: ");
          Serial.println((int32_t)Q_avg_val); 

          uint8_t bufferSend[7];
          bufferSend[0] = I_avg_val_20 & 0xFF;
          bufferSend[1] = (I_avg_val_20 >> 8) & 0xFF;
          bufferSend[2] = ((I_avg_val_20 >> 16) & 0x0F) | ((Q_avg_val_20 & 0x0F) << 4);
          bufferSend[3] = (Q_avg_val_20 >> 4) & 0xFF;
          bufferSend[4] = (Q_avg_val_20 >> 12) & 0xFF;
          bufferSend[5] = 0x03;            
          bufferSend[6] = nbFreqRecieved;  
          
          if (deviceConnected) {
            pFrame7byteCharacteristic->setValue(bufferSend, sizeof(bufferSend));
            pFrame7byteCharacteristic->notify();
          }
        }
      }
    }
  }
  
  if ((valuesSent == numberReadFifo) || !enableAverageTransmission) {
    Serial.println("Transmission end");
    shut_down();
    enableAverageTransmission = 0;

    nbFreqRecieved++;
    if (nbFreqRecieved == nbFrequenciesExpected) {
      Serial.println("Calibration finished successfully");
      uint8_t bufferSend[7];
      bufferSend[0] = 0x00;
      bufferSend[1] = 0x00;
      bufferSend[2] = 0x00;
      bufferSend[3] = 0x00;
      bufferSend[4] = 0x00;
      bufferSend[5] = 0x0F;
      bufferSend[6] = 0xFF;
      
      if (deviceConnected) {
        pFrame7byteCharacteristic->setValue(bufferSend, sizeof(bufferSend));
        pFrame7byteCharacteristic->notify();
      }
      enableSweep = 0;
    }
  }
}