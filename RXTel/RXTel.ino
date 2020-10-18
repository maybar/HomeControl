/** \file RXTel.ino
 * RF gateway for Arduino projects
 * This program runs in the GATEWAY hardware.
 * Miguel A. 23 Octubre 2016
 * Arduino Digispark ATTINY85
 * Default -16.5mhz
 */

/*! \mainpage Telemetry module is used for general comunication issues.
\section Pinout Introduction
 P0 OUTPUT DATA TX
 P2 INPUT DATA RX

*/
#include <VirtualWire.h>
#include <DigiCDC.h>    //To use the terminal
 
#define LEN_BUFFER_RX  7 
#define PIN_TX  0
#define PIN_RX  2
int ledPin = 1;
/** \fn void setup
	It initializes the variables and configures the hardware
*/
void setup()
{
 
  SerialUSB.begin(); 
  vw_set_tx_pin(PIN_TX);
  vw_set_rx_pin(PIN_RX);

  vw_setup(1000);   // Bits per sec
  vw_rx_start();       // Start the receiver PLL running 

  pinMode(ledPin, OUTPUT);
  digitalWrite(PIN_TX, HIGH); //!!Muy importante

}

void loop()
{
  uint8_t radio_msg[LEN_BUFFER_RX];  //Data transmission buffer
  uint8_t buflen = sizeof(radio_msg);

  tx();
  //if (vw_get_message(radio_msg, &buflen)) // Non-blocking
  //if (vw_have_message())
  {
	int i;
    //vw_rx_stop();
    //digitalWrite(ledPin, true); // Flash a light to show received good message
    // Message with a good checksum received, dump it.
    //SerialUSB.print(F("Got: "));
    //SerialUSB.print(buflen);
    //SerialUSB.refresh();
    //for (i = 0; i < buflen; i++)
    //{
        //SerialUSB.print(radio_msg[i], HEX);
        //SerialUSB.print(F(" "));
    //}
    //SerialUSB.println("-");
    //SerialUSB.delay(250);
    //digitalWrite(ledPin, false);
    //vw_rx_start();
  }
  //else
  {
    //digitalWrite(ledPin, true);
    //SerialUSB.delay(1000);
    //digitalWrite(ledPin, false);
    //SerialUSB.delay(1000);
    //SerialUSB.println("-");
  }
}

void tx()
{
  static int cont = 0;
  uint8_t msg[10]= {'H','o','l','a'};
  digitalWrite(ledPin, true);
  msg[0] = cont;
  vw_send((uint8_t *)msg, 1);
  vw_wait_tx(); // Wait until the whole message is gone
  digitalWrite(ledPin, false);
  SerialUSB.delay(1000);
  cont++;
}
   /*
   if you don't call a SerialUSB function (write, print, read, available, etc) 
   every 10ms or less then you must throw in some SerialUSB.refresh(); 
   for the USB to keep alive - also replace your delays - ie. delay(100); 
   with SerialUSB.delays ie. SerialUSB.delay(100);
   */
