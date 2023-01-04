#include <Arduino.h>

/*****************************************
  ESP32 GPS VKEL 9600 Bds
This version is for T22_v01 20190612 board
As the power management chipset changed, it
require the axp20x library that can be found
https://github.com/lewisxhe/AXP202X_Library
You must import it as gzip in sketch submenu
in Arduino IDE
This way, it is required to power up the GPS
module, before trying to read it.
Also get TinyGPS++ library from:
https://github.com/mikalhart/TinyGPSPlus
******************************************/

#include <LoRa.h>
#include <SPI.h> // include libraries
#include <TinyGPS++.h>
#include <axp20x.h>

// lora section
const int SCK_pin = 5;   // GPIO5  -- SX1278's SCK
const int MISO_pin = 19; // GPIO19 -- SX1278's MISnO
const int MOSI_pin = 27; // GPIO27 -- SX1278's MOSI
const int SS_pin = 18;   // GPIO18 -- SX1278's CS
const int RST_pin = 14;  // GPIO14 -- SX1278's RESET
const int DI0_pin = 26;  // GPIO26 -- SX1278's IRQ(Interrupt Request)
// #define BAND 868E6

const long lora_frequency_BAND = 850125000; // LoRa Frequency

TinyGPSPlus gps;
HardwareSerial GPS(1);
AXP20X_Class axp;

void setup()
{
  Serial.begin(115200);
  Wire.begin(21, 22);
  if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS))
  {
    Serial.println("AXP192 Begin PASS");
  }
  else
  {
    Serial.println("AXP192 Begin FAIL");
  }
  axp.EnableCoulombcounter();
  axp.enableChargeing(1);
  axp.adc1Enable(AXP202_VBUS_VOL_ADC1 |
                     AXP202_VBUS_CUR_ADC1 |
                     AXP202_BATT_CUR_ADC1 |
                     AXP202_BATT_VOL_ADC1 |
                     AXP202_ACIN_VOL_ADC1 |
                     AXP202_ACIN_CUR_ADC1 |
                     AXP202_APS_VOL_ADC1 |
                     AXP202_TS_PIN_ADC1,
                 true);
  // axp.adc2Enable(AXP202_);
  // axp.enableDC2VRC()
  axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
  axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
  axp.setDCDC1Voltage(3300);
  GPS.begin(9600, SERIAL_8N1, 34, 12); // 17-TX 18-RX

  SPI.begin(SCK_pin, MISO_pin, MOSI_pin, SS_pin);
  LoRa.setPins(SS_pin, RST_pin, DI0_pin);
  if (!LoRa.begin(lora_frequency_BAND))
  {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
  LoRa.setFrequency(850125000);
  //LoRa.setSignalBandwidth()
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (GPS.available())
      gps.encode(GPS.read());
  } while (millis() - start < ms);
}

void loop()
{
  static uint8_t counter = 0;
  Serial.print("getBattVoltage  : ");
  Serial.println(axp.getBattVoltage(), 5);

  Serial.print("getBattChargeCurrent  : ");
  Serial.println(axp.getBattChargeCurrent(), 5);

  Serial.print("getBattDischargeCurrent  : ");
  Serial.println(axp.getBattDischargeCurrent(), 5);
  Serial.print("getBattChargeCoulomb  : ");
  Serial.println(axp.getBattChargeCoulomb(), 5);
  Serial.print("getBattDischargeCoulomb  : ");
  Serial.println(axp.getBattDischargeCoulomb(), 5);
  Serial.print("getCoulombData  : ");
  Serial.println(axp.getCoulombData(), 5);
  Serial.print("getSysIPSOUTVoltage  : ");
  Serial.println(axp.getSysIPSOUTVoltage(), 5);

  Serial.print("getAcinVoltage  : ");
  Serial.println(axp.getAcinVoltage(), 5);
  Serial.print("getAcinCurrent  : ");
  Serial.println(axp.getAcinCurrent(), 5);

  Serial.print("getVbusVoltage  : ");
  Serial.println(axp.getVbusVoltage(), 5);
  Serial.print("getVbusCurrent  : ");
  Serial.println(axp.getVbusCurrent(), 5);

  Serial.print("getTemp  : ");
  Serial.println(axp.getTemp(), 5);

  Serial.print("Latitude  : ");
  Serial.println(gps.location.lat(), 5);
  Serial.print("Longitude : ");
  Serial.println(gps.location.lng(), 4);
  Serial.print("Satellites: ");
  Serial.println(gps.satellites.value());
  Serial.print("Altitude  : ");
  Serial.print(gps.altitude.meters());
  Serial.println("M");
  Serial.print("Time      : ");
  Serial.print(gps.time.hour());
  Serial.print(":");
  Serial.print(gps.time.minute());
  Serial.print(":");
  Serial.println(gps.time.second());
  Serial.print("Speed     : ");
  Serial.println(gps.speed.kmph());
  Serial.println("**********************");

  // send packet
  LoRa.beginPacket();
  LoRa.print("\0\0\0");
  LoRa.print("hello ");
  LoRa.print(counter);
  LoRa.endPacket();

  ++counter;

  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}
