/*
 * Copyright (c) 2025 FAB Controls Ltd.
 * www.fabcontrols.co.uk
 *
 * All rights reserved.
 *
 * This software is the property of FAB Controls Ltd. and is protected by
 * copyright law. It is intended for use on the
 * Wurth Elektronix Oceanus Evaluation Kit
 * https://www.we-online.com/en/components/products/OCEANUS-I
 *
 */

#include <Arduino.h>
#include <STM32LoRaWAN.h>
#include "secrets.h"

// Initalise Modem
LoRaModem modem;

/* Send LoRaWAN Packet Timer */
#define APP_TX_DUTYCYCLE 60000

// Set appEui and appKey in secrets.h
String appEui = SECRET_APP_EUI;
String appKey = SECRET_APP_KEY;

// Obtain time stamp for build date
#define BUILD_DATE TIME_STAMP

// Eval Board LED Setup
const int ledPin1 = PA7;
const int ledPin2 = PA4;

// P1 Connector Pin Definitions

// Digital Outputs / Inputs
const int PIN1 = PA1; //  Pin 1
const int PIN2 = PA5; //  Pin 2
const int PIN3 = PA6; //  Pin 3
const int PIN5 = PA8; //  Pin 5
const int PIN7 = PB6; //  Pin 7

// AD2 Inputs
const int PIN4 = PB2;  // Pin 4
const int PIN6 = PA12; // Pin 6
const int PIN8 = PA11; // Pin 8

// Downlink command definitions
const uint8_t CMD_LED1_ON_LED2_OFF = 0xaa;
const uint8_t CMD_LED2_ON_LED1_OFF = 0xbb;
const uint8_t CMD_LEDS_ON = 0xcc;
const uint8_t CMD_LEDS_OFF = 0xdd;
const uint8_t CMD_REBOOT = 0xee;

// Switch LEDS on and off from LoRaWAN downlink
void leds(const uint8_t *control, size_t len)
{
  if (len == 0)
  {
    return; // No command to process
  }

  switch (control[0])
  {
  case CMD_LED1_ON_LED2_OFF:
    Serial.println("Led 1 on, 2 Off");
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, LOW);
    Serial.println("=====================================");
    break;

  case CMD_LED2_ON_LED1_OFF:
    Serial.println("Led 2 on, 1 Off");
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, HIGH);
    Serial.println("=====================================");
    break;

  case CMD_LEDS_ON:
    Serial.println("Led 1 on, 2 On");
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, HIGH);
    Serial.println("=====================================");
    break;

  // All LEDS Off
  case CMD_LEDS_OFF:
    Serial.println("All OFF");
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, LOW);
    Serial.println("=====================================");
    break;

  // Send Remote Reboot Command
  case CMD_REBOOT:
    Serial.println("Reboot");
    Serial.println("=====================================");
    HAL_NVIC_SystemReset();
    // This break will not be reached, but is good practice for completeness.
    break;
  }
}

void rx_packet()
{
  int packetSize = modem.available();
  if (packetSize > 0)
  {
    uint8_t rcv[64];
    size_t len = modem.read(rcv, sizeof(rcv));

    Serial.print("Received: ");
    for (size_t j = 0; j < len; j++)
    {
      Serial.print(rcv[j] >> 4, HEX);
      Serial.print(rcv[j] & 0xF, HEX);
      Serial.println(" ");
    }
    Serial.println("=====================================");
    Serial.print("Downlink Port: "), Serial.println(modem.getDownlinkPort());
    Serial.print("Downlink RSSI: "), Serial.println(modem.getDownlinkRssi());
    Serial.print("Downlink SNR: "), Serial.println(modem.getDownlinkSnr());

    // Process the received command
    leds(rcv, len);
  }
}

// Send LoRaWAN Packet - Device is class A so must wait for RX window after TX
void send_packet()
{

  char payload[27] = {0}; /* packet to be sent */
  /* prepare the Tx packet : get date and format string */
  sprintf(payload, "Wurth Oceanus Eval Kit");

  modem.setPort(10);
  modem.beginPacket();
  modem.write(payload, strlen(payload));

  // Serial.println(payload);

  if (modem.endPacket() == (int)strlen(payload))
  {
    Serial.println("=====================================");
    Serial.println("Sent packet");
    Serial.println("=====================================");
  }
  else
  {
    Serial.println("Failed to send packet");
  }

    // Wait for RX Window
    rx_packet();
  }

  // Read the Analog Inputs and print to Serial
  void read_analog()
  {
    Serial.println();
    Serial.println("--- Analog Readings ---");

    // Read and print the raw value for PIN4
    int pin4Value = analogRead(PIN4);
    Serial.print("PIN4 (PB2) Raw Value: ");
    Serial.println(pin4Value);

    // Read and print the raw value for PIN6
    int pin6Value = analogRead(PIN6);
    Serial.print("PIN6 (PA12) Raw Value: ");
    Serial.println(pin6Value);

    // Read and print the raw value for PIN8
    int pin8Value = analogRead(PIN8);
    Serial.print("PIN8 (PA11) Raw Value: ");
    Serial.println(pin8Value);
  }

  // Inital Debug Header
  void debug_header(String status)
  {

    Serial.println("=====================================");
    Serial.println("© FAB Controls Ltd - 2025");

    Serial.println("Wurth Elektronix Oceanus Evaluation Kit");
    Serial.print("LoRaWAN Status: "), Serial.println(status);

    time_t rawtime = BUILD_DATE;
    struct tm *timeinfo = localtime(&rawtime);
    char buffer[80];

    // Example format string: "Day_of_week Month Day Year Hour:Minute:Second"
    strftime(buffer, sizeof(buffer), "%d %B %Y ", timeinfo);
    Serial.print("Build Date: ");
    Serial.println(buffer);

    Serial.print("Device EUI: ");
    Serial.println(modem.deviceEUI());

    Serial.println("===================================== \n");
  }

  void setup()
  {

    // Initialize Serial for debugging
    Serial.begin(115200);
    // while (!Serial);

    // Initialize Eval Board Outputs
    pinMode(ledPin1, OUTPUT);
    pinMode(ledPin2, OUTPUT);
    pinMode(PIN1, OUTPUT);
    pinMode(PIN2, OUTPUT);
    pinMode(PIN3, OUTPUT);
    pinMode(PIN5, OUTPUT);
    pinMode(PIN7, OUTPUT);

    // Set A/D Resolution to 12 bits for more detailed readings
    // analogReadResolution(12);

    // Initialize LoRaWAN connectivity
    if (!modem.begin(EU868))
    {
      Serial.println("Failed to start module");
      while (1)
      {
      }
    };

    // Initialize the LoRaWAN modem
    int connected = modem.joinOTAA(appEui, appKey);

    // Connect Loop
    if (connected)
    {

      // Call Debug Header
      debug_header("Network Joined");
    }
    else
    {
      // Call Debug Header
      debug_header("Network Join failed");
      // read_analog_inputs();
      while (true)
        ; /* infinite loop */
      // Add code to reboot
    }
  }

  void loop()
  {

    // Send a LoRaWAN packet
    send_packet();

    // Read the Analog Inputs
    read_analog();

    delay(2000); // Kept original delay, consider replacing with non-blocking timer
  }
