/*
 * Remote Temperature Sensor
 * Arduino Nano w/ATmega168
 * PIR: Tiempo de rearme ALTO: 3:15 minutos  (Pot derecha)
 * PIR: Sensibilidad BAJA (Pot. Izquierda)
 * Transmiter: STX882 ASK 433 MHz
 * Use Timer 2: 8-bit, PWM on two pins 17 and 5 for interrupts
 * 
 */
/// Arduino                         Transmitter
///  GND------------------------------GND
///  D12------------------------------Data
///  5V-------------------------------VCC

/// Arduino                         TH11
///  GND------------------------------GND
///  D4-------------------------------Data
///  5V-------------------------------VCC

/// Arduino                         Pir
///  GND------------------------------GND
///  D2-------------------------------Data
///  5V-------------------------------VCC

 #include <dht11.h>              //DHT11 library
 //#include <TimerOne.h>
#include <MsTimer2.h>
 #include <avr/sleep.h>
 #include <avr/power.h>
#include <VirtualWire.h>    // radio communication

#define LEN_BUFFER_TEMP  5
#define LEN_BUFFER_TX  7
#define RTS_ID 1

dht11 DHT;

// the input pin (for PIR sensor) INTERRUPT
#define PIR_PIN 2    
//Pin 4 of Arduino to Data of DHT11
#define DHT11_PIN 4    
#define TX_PIN 12
#define PIN_BATTERY_MONITOR 0    // analog pin to monitor

//-------------------
#define TIMER_PERIOD_MS 5000
double const TIME_REPORT_MS = 10000;   //1hora
double const TIME_SENSOR_MS = 60000;  //1 minuto

int ledPin = 13;                // choose the pin for the LED

int pirState = LOW;             // we start, assuming no motion detected
int i_temp = 0;
int i_humi = 0;
int val = 0;                    // variable for reading the pin status
int local_power = 0;
int status_sensor = DHTLIB_OK;
volatile bool b_flag_read_pir = true;
volatile bool b_flag_timer = true;
 
void setup() {
  pinMode(ledPin, OUTPUT);      // declare LED as output
  pinMode(PIR_PIN, INPUT);     // declare sensor as input
  pinMode(DHT11_PIN, INPUT_PULLUP);     // declare sensor as input  
  pinMode(ledPin, OUTPUT);
 
  Serial.begin(9600);
  Serial.println("Setup");
  
  check_sensor();
  delay(100);
  for (int y=0;y<5; ) 
  {
    read_sensor(true);  // llena el buffer
    if(status_sensor==0) y++;
    delay(100);    //Espera antes de leer otro dato
  }
  
 MsTimer2::set(TIMER_PERIOD_MS,timer_isr); // set a timer of length 8 seconds 
 MsTimer2::start();
  
  // Initialise the IO and ISR
  vw_set_tx_pin(TX_PIN);
  vw_setup(2000);   // Bits per sec
  Serial.println("init radio driver started");
         
  //Configura la referencia del conversor analogico para el sensor de voltaje  
  analogReference(INTERNAL);      // set reference to internal (1.1V)
  analogRead(PIN_BATTERY_MONITOR);  // allow the ADC to settle
  delay(10);
}
 
void loop()
{
  static double i_msec=TIME_REPORT_MS+1;    //mseconds counter
  static double i_counter=0;    //counter
  byte radio_msg[LEN_BUFFER_TX];  //Data transmission buffer
  //Serial.print(".");
  bool b_send = false;    // enviar


  i_counter++;
  if (b_flag_timer)
  {
    i_msec=i_msec+TIMER_PERIOD_MS;    
    b_flag_timer=false;
  }
  if (i_msec >= TIME_REPORT_MS )  
  {
    //b_timer=true;
    b_send=true;
    Serial.println("timer");
    
  }
  
  // If woke up by PIR
  if (b_flag_read_pir || b_send){
    bool b_change_value1 = motion_detect();
    b_send = (b_change_value1)?true:b_send;
  }
  if (b_flag_read_pir) 
  {
    Serial.println("pir");
    b_flag_read_pir = false;
  }
  // cada minuto comprueba la temperatura
  bool b_sensor_time = (i_counter >TIME_SENSOR_MS);
  if (b_sensor_time || b_send)
  {
   // temperature and humidity
   bool b_change_value2 = read_sensor(b_sensor_time);
   b_send = (b_change_value2)?true:b_send;
  }
  if (b_sensor_time){
    i_counter = 0;
    Serial.println("sensor");
  }
   
   //Envia datos si detecta movimiento o cambia los datos metereologicos
   if (b_send)
   {
     i_msec = 0;
     digitalWrite(ledPin, HIGH);  // turn LED ON
     local_power = batteryMv(PIN_BATTERY_MONITOR);
     
     radio_msg[0]= RTS_ID;
     radio_msg[1]= pirState;
     radio_msg[2]= i_temp;
     radio_msg[3]= i_humi;
     radio_msg[4]= local_power/100;
     radio_msg[5]= status_sensor;
     Serial.print("TX: ");
     int chksum=0;
     for (int x=0;x< LEN_BUFFER_TX-1;x++)
     {
       chksum = chksum+radio_msg[x];
       Serial.print(radio_msg[x]);
       Serial.print("\t");
     }
     Serial.println();
     radio_msg[6]= chksum;
     vw_send((uint8_t *)radio_msg, LEN_BUFFER_TX);
     vw_wait_tx(); // Wait until the whole message is gone
     digitalWrite(ledPin, LOW);  // turn LED OFF
   }
   delay(20);
   sleep_now(); // Dormir 
   
} 

//Detector PIR cambio de estado
void ext_pir_isr()
{
  detachInterrupt(digitalPinToInterrupt(PIR_PIN));
  b_flag_read_pir = true;
}


// Rutina de inbterrupcion del timer
void timer_isr()
//ISR(TIMER1_OVF_vect)
{
  /* set the flag. */
   if(b_flag_timer == false)
   {
     b_flag_timer = true;
   }
}


// Dormior en modo bajo consumo
void sleep_now()
{
  attachInterrupt(digitalPinToInterrupt(PIR_PIN), ext_pir_isr, CHANGE);   //Active the INT for PIR
  
  set_sleep_mode(SLEEP_MODE_IDLE);  //sleep mode
  
  sleep_enable();
  
  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer1_disable();  
  //power_timer2_disable(); //dejar el timer activo
  power_twi_disable(); 

  // enter in sleep mode
  sleep_cpu();
 
  
  //wake up
  sleep_disable();
  power_all_enable();
  
}

void check_sensor()
{
  int check;
  Serial.print("DHT11 STATUS - ");
  check = DHT.read(DHT11_PIN);
   //check status
   switch (check)
   {
      case DHTLIB_OK:
         Serial.println("OK");
         break;
      case DHTLIB_ERROR_CHECKSUM:
         Serial.println("Checksum error");
         break;
      case DHTLIB_ERROR_TIMEOUT:
         Serial.println("Timeout error");
         break;
      default:
         Serial.println("Unknown error");
         break;
   }
}
//! Lee datos de sensor de temperatura
//! Si b_sensor_time==true almacena la temperatura en el buffer
bool read_sensor(bool b_sensor_time)
{
  static int a_temp_buffer[LEN_BUFFER_TEMP];  //buffer
  bool b_changed=false;
   status_sensor = DHT.read(DHT11_PIN);
   if (status_sensor == DHTLIB_OK)
   {
     // Mete en el buffer circular
     if(b_sensor_time)  // si ha pasado 1 periodo de lectura
     {
       for (int x=(LEN_BUFFER_TEMP-1);x>0;x--){
         a_temp_buffer[x] = a_temp_buffer[x-1];
       }
       a_temp_buffer[0] = DHT.temperature;
     }
     // Media
     float f_media_temp =0;
     for (int x=0;x<LEN_BUFFER_TEMP;x++) 
     {
       f_media_temp = f_media_temp + a_temp_buffer[x];
       //Serial.print(a_temp_buffer[x]);
       //Serial.print(" ");
     }
     f_media_temp = f_media_temp/LEN_BUFFER_TEMP;
     
     int i_media_temp = int(f_media_temp);
     int decimal = ((f_media_temp-i_media_temp)*10);
     if (decimal >= 5)
       i_media_temp = i_media_temp+1;  // aproximar al superior
       
     Serial.println(f_media_temp);
     //! Detecta cambio con la lectura anterior
     if (i_media_temp != i_temp) 
     {
       b_changed = true;
     }
     i_temp = i_media_temp;
     i_humi = DHT.humidity;
   } 
   else Serial.println("Error sensor");
  return b_changed; 
}

bool motion_detect()
{
  bool b_detected = false; 
  val = digitalRead(PIR_PIN);  // read input value
  if (val == HIGH) {            // check if the input is HIGH
    //digitalWrite(ledPin, HIGH);  // turn LED ON
    if (pirState == LOW) {
      // we have just turned on
      //Serial.println("Motion detected!");
      // We only want to print on the output change, not state
      pirState = HIGH;
      b_detected = true;
    }
  } else {
    //digitalWrite(ledPin, LOW); // turn LED OFF
    if (pirState == HIGH){
      // we have just turned of
      //Serial.println("Motion ended!");
      // We only want to print on the output change, not state
      pirState = LOW;
      b_detected = true;
    }
  }
  return b_detected;
}

// return the voltge on the given pin in millivolts
// see text for voltage divider resistor values
int  batteryMv(int pin )
{
#if defined(__AVR_ATmega32U4__) // is this a Leonardo board?
  const long INTERNAL_REFERENCE_MV = 2560; // leo reference is 2.56 volts
#else
  const long INTERNAL_REFERENCE_MV = 1100; // ATmega328 is 1.1 volts
#endif  
  const float R1 = 10.0;  // voltage divider resistors values, see text
  const float R2 = 1.0;    
  const float DIVISOR = R2/(R1+R2); 

  analogReference(INTERNAL);      // set reference to internal (1.1V)
  analogRead(pin);  // allow the ADC to settle
  delay(10);

  int value = 0;
  for(int i=0; i < 3; i++) {    
    value = value + analogRead(pin);
  }
  value  = value / 3; // get the average of 3 readings
  int mv = map(value, 0,1023, 0, INTERNAL_REFERENCE_MV / DIVISOR );

  analogReference(DEFAULT); // set the reference back to default (Vcc)
  analogRead(pin); // just to let the ADC settle ready for next reading
  delay(10); // allow reference to stabalise

  return mv;
}
