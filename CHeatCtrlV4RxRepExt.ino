// Heat Receiver control with repeater Transmit
// S.Beadle, 1-4-17, 11/12/17, 16/12/18, 8/1/20, 8/12/23
// Radiohead 1.112
// Arduino Pro Mini
#define BAUDRATE 9600

// Radio input is 11
// Radio output is 12
// Pin 9 is ptt not connected

// Option defines
//#define CHURCHSWITCH
#define ONESWITCH
//#define DOORSWITCH

// Select options
#ifdef CHURCHSWITCH
#define CHAN1COMMAND 'R' // Command - first channel is 'R'
#define RELAYON 1 // Relay 1 Pin 10 is output off LOW=0, and HIGH=1
#define RELAY2ON 0 // Relay 2 Pin 8 is output off LOW=0, and HIGH=1, and PULSE=2
#define REPEATENERGENIE 0 // If not zero This code will repeat signal on Tx to Energenie output
#define REPEATASK 1 // If not zero This code will repeat signal on Tx to R0 and R1 commands
#define WATCHDOGTURNOFF 1 // This will make the watchdog turn power off when it activates
#define STARTSTATE1 false // Start states
#define STARTSTATE2 false // Start states
#endif

#ifdef ONESWITCH
#define CHAN1COMMAND 'R' // Command - first channel is 'R'
#define RELAYON 0 // Relay 1 Pin 10 is output off LOW=0, and HIGH=1
#define RELAY2ON 0 // Relay 2 Pin  8 is output off LOW=0, and HIGH=1, and PULSE=2
#define REPEATENERGENIE 0 // If not zero This code will repeat signal on Tx to Energenie output
#define REPEATASK 1 // If not zero This code will repeat signal on Tx to R0 and R1 commands
#define WATCHDOGTURNOFF 0 // If not zero  This will make the watchdog turn power off when it activates
#define STARTSTATE1 false // Start states
#define STARTSTATE2 false // Start states
#endif

#ifdef DOORSWITCH
#define CHAN1COMMAND 'S' // Command - Second: set to 'S' for door power and set relay 2 'T' for bell
#define DOORSWITCH 2 // Do not define this one, comment out to ignore the door
#define DOORSWITCHDELAY 15000L // Input Microswitch, Now 15 second delay before bell
#define RELAYON 0 // Relay 1 Pin 10 is output off LOW=0, and HIGH=1
#define RELAY2ON 2 // Relay 2 Pin  8 is output off LOW=0, and HIGH=1, and PULSE=2
#define REPEATENERGENIE 0 // If not zero This code will repeat signal on Tx to Energenie output
#define REPEATASK 0 // If not zero This code will repeat signal on Tx to R0 and R1 commands
#define WATCHDOGTURNOFF 0 // If not zero  This will make the watchdog turn power off when it activates
#define STARTSTATE1 true // Start states
#define STARTSTATE2 false // Start states
#endif

// Relay 1 output is 10, echo to PIN 13 LED, flash on Rx
#define CHAN1PIN 10

// Relay 2 output is 8. Needs to be defined. Connected to a bell.
#define CHAN2PIN 8
#define CHAN2COMMAND 'T'

// LED is pin 13
#define LEDPIN 13


// Tx Parameters
#define DATA_L       615  // Long pulse
#define DATA_S       205  // Short pulse
#define MESSAGEGAP  6000  // Intra messags
#define MSGCOUNT      20  // 20 message bursts
#define CHECKPERIOD 1000  // 1 sec between checks
#define MAXCOUNT      30  // loop this number before retransmit - should be 30
#define INTERMSG     250  // 1/4s sec between virtual wire messages
#define RETXWAIT    2000  // Wait 1 sec between rx and re-tx
#define RETXMIN     5000  // Don't send another TX if within this period of the last
#define RETXMAX    15000  // Don't send another TX if within this period of the last
#define FLASHINT     200  // Flash check interval
#define WATCHDOG  300000  // 5 min Watchdog interval - will turn relay off if no receipt within this time

#include <RH_ASK.h>
#include <SPI.h>

void sendOne() { //Short high, long low
  digitalWrite(12, HIGH);
  delayMicroseconds(DATA_L);
  digitalWrite(12, LOW);
  delayMicroseconds(DATA_S);
}

void sendZero() { //Long high, short low
  digitalWrite(12, HIGH);
  delayMicroseconds(DATA_S);
  digitalWrite(12, LOW);
  delayMicroseconds(DATA_L);
}

// Send this number of messages consecutively
void sendcode( const char *msg) {
  int i;
  for (i = 0; i < MSGCOUNT; i++) { 
    int j;
    for (j=0; j<25; j++) { // Fixed bit count
      int c = msg[j];
      if (c == '0') {
        sendOne();
      }
      else {
        sendZero();
      }
    }
    delayMicroseconds(MESSAGEGAP);
  }
}

//RH_ASK driver constructor
// RH_ASK(uint16_t speed = 2000, uint8_t rxPin = 11, uint8_t txPin = 12, uint8_t pttPin = 10, bool pttInverted = false);
RH_ASK driver(2000, 11, 12, 9);

void setup()
{
  Serial.begin(BAUDRATE); // Debugging only
  Serial.println("Startup");

  pinMode(LEDPIN, OUTPUT);        // LED
  digitalWrite(LEDPIN, true);

  if (!driver.init())
  {
     Serial.println("Radio init failed");
     digitalWrite(LEDPIN, true);
  } else {
    Serial.println("Radio setup OK");
  }

  pinMode( CHAN1PIN, OUTPUT);        // Relay Off
  pinMode( CHAN2PIN, OUTPUT);        // Relay Off

  Serial.println( "Setup options:");
  Serial.print  ( "Relay 1 Command: ");
  Serial.println( CHAN1COMMAND);
  Serial.println( RELAYON ? "High Level Relay 1" : "Low Level Relay 1");
  Serial.print  ( "Relay 2 Command: ");
  Serial.println( CHAN2COMMAND);
  Serial.println( RELAY2ON == 2 ? "Relay 2 Pulse" : ( RELAY2ON ? "High Level Relay 2" : "Low Level Relay 2" ) );
  Serial.println( REPEATENERGENIE ? "Repeat to Energenie" : "No Repeat to Energenie");
  Serial.println( REPEATASK ? "Repeat to ASK" : "No Repeat to ASK");
  Serial.println( WATCHDOGTURNOFF ? "Watchdog trigger power off" : "Watchdog does not affect power");
  digitalWrite(LEDPIN, false);

#ifdef DOORSWITCH
  pinMode( DOORSWITCH, INPUT_PULLUP);
#endif
  Serial.println("Startup done");
}

// Output starts off
boolean currentState = STARTSTATE1;
boolean currentState2 = STARTSTATE2;

// Retransmit
boolean retxneeded = false;
unsigned long lastcheck = 0;
long nextretranstime = RETXMIN;

// Flashing
unsigned long lastflash = 0;
int flashstate = 0;

// Watchdog
unsigned long lastwatchdog = 0;
// Start with inactive watchdog
boolean watchdogactive = false;

// Door
bool DoorOpen = false;
unsigned long DoorOpenTime = 0L;
unsigned long DoorClosedTime = 0L;
int DoorOnCount = 0;

void loop()
{
  bool lastState = currentState;
  bool lastState2 = currentState2;

  unsigned long checktime = millis();

  uint8_t buf[12];
  uint8_t buflen = sizeof(buf);
  uint8_t buf2[12];
  uint8_t buflen2 = sizeof(buf2);

  if (driver.recv(buf, &buflen))
  {
    int i;
    buflen2 = buflen;
    strncpy( buf2, buf, buflen2);
    
    // Flash a light to show received good message
    digitalWrite(LEDPIN, true); 
    // Message with a good checksum received, dump it.
    Serial.print("Rxv: ");
    Serial.print( buflen2);
    Serial.print(", ");
    
    for (i = 0; i < buflen2; i++)
    {
        Serial.print(buf2[i], HEX);
        Serial.print(" ");
    }
    Serial.print(" / ");
    for (i = 0; i < buflen2; i++)
    {
      if (buf2[i]>31 && buf2[i]<128) {
        Serial.print((char)buf2[i]);
      } else {
        Serial.print('_');
      }
    }
    Serial.println("");
    digitalWrite(LEDPIN, false);

    boolean messageok = false;
    if (buflen2 == 2)
    {
      if(buf2[0] == CHAN1COMMAND && buf2[1] == '0'){
        Serial.println("Relay 1 off");
        currentState = false;
        messageok = true;
      }
      if(buf2[0] == CHAN1COMMAND && buf2[1] == '1'){
        Serial.println("Relay 1 on");
        currentState = true;
        messageok = true;
      }
      if(buf2[0] == CHAN2COMMAND && buf2[1] == '0'){
        Serial.println("Relay 2 off");
        currentState2 = false;
        messageok = true;
      }
      if(buf2[0] == CHAN2COMMAND && buf2[1] == '1'){
        Serial.println("Relay 2 on");
        currentState2 = true;
        messageok = true;
      }
    }
    if (messageok) {
      lastwatchdog = checktime;
      watchdogactive = false;
      
#if (REPEATENERGENIE == 1) || (REPEATASK == 1)
      retxneeded = true;
      lastcheck = checktime;
      nextretranstime = random(RETXMIN, RETXMAX);
      Serial.print("Sched Re-tx at: ");
      Serial.println( nextretranstime);
#endif
    }
  }
  
  // Write output every time we loop
  if (currentState) {
    digitalWrite( CHAN1PIN, RELAYON == 0); // ON = high if LOW Relay (output high leads to relay on)
  } else {
    digitalWrite( CHAN1PIN, RELAYON == 1); // OFF = high if HIGH Relay
  }
#if RELAY2ON == 2
  if (currentState2 && !lastState2) {
    RingBell( 500);
  }
#else
  if (currentState2) {
    digitalWrite( CHAN2PIN, RELAY2ON == 0); // ON = high if LOW Relay (output high leads to relay on)
  } else {
    digitalWrite( CHAN2PIN, RELAY2ON == 1); // OFF = high if HIGH Relay
  }
#endif

#if (REPEATENERGENIE == 1) || (REPEATASK == 1)
  // If we need to retransmit and the time is ready (& not too close to bootup)
  if ((retxneeded) && (checktime > (6L * RETXMAX))) {
    if (checktime - lastcheck > nextretranstime) {
      sendCode( currentState);
      retxneeded = false;
    }
    if (checktime < lastcheck) {
      lastcheck = checktime; // Handle rollover
    }
  }
#endif

  // If checktime greater than lastflash and half a second
  if (checktime - lastflash > FLASHINT) {
    // Rapid flash if watchdog is active
    if (watchdogactive) {
      if (flashstate == 0) {
        digitalWrite( LEDPIN, HIGH);
      }
      if (flashstate == 1) {
        digitalWrite( LEDPIN, LOW);
      }
      flashstate++;
      if (flashstate >= 2) {
        flashstate = 0;
      }
    } else {
      // If relay is on
      if (currentState) {
        if (flashstate == 0) {
          digitalWrite( LEDPIN, LOW);
        }
        if (flashstate == 1) {
          digitalWrite( LEDPIN, HIGH);
        }
        flashstate++;
        if (flashstate >= 6) {
          flashstate = 0;
        }
      } else {
        // off
        if (flashstate == 0) {
          digitalWrite( LEDPIN, HIGH);
        }
        if (flashstate == 1) {
          digitalWrite( LEDPIN, LOW);
        }
        flashstate++;
        if (flashstate >= 10) {
          flashstate = 0;
        }
      }
    }
    lastflash = checktime;
  }  
  if (checktime < lastflash) {
    lastflash = checktime; // Handle rollover
  }

  // Watchdog
  if (checktime - lastwatchdog > WATCHDOG) {
#if WATCHDOGTURNOFF == 1
    // If on then turn it off
    if (currentState) {
      currentState = false;
      // retxneeded = true; Should not cascade watchdog
      Serial.println("WATCHDOG TURN OFF");
    } else {
      Serial.println("WATCHDOG NO SIGNAL WHILE OFF");
    }
#endif
    watchdogactive = true;
    lastwatchdog = checktime;
  }
  if (checktime < lastwatchdog) {
    lastwatchdog = checktime; // Handle rollover
  }

#ifdef DOORSWITCH
  // Read door switch - 1 is Open
  bool DoorOpenNow = digitalRead( DOORSWITCH);
  if ( DoorOpenNow != DoorOpen) {
    if ( DoorOpenNow ) {
      Serial.println( "Door Open");
      DoorOpenTime = checktime;
      DoorOnCount++;
    } else {
      Serial.println( "Door Closed");
      DoorClosedTime = checktime;
    }
    DoorOpen = DoorOpenNow;
  }
  // Change to 4 presses to neutralise
  if ( DoorOpenNow && ( checktime - DoorOpenTime > DOORSWITCHDELAY ) && (DoorOnCount < 4) ) {
    Serial.println( "Door Bell Alert");
    RingBell( 150); // Shorten from 200
    // repeat every interval
    DoorOpenTime = checktime;
    // BUT Neutralise the above, and only sound once
    DoorOnCount = 4;
  }
  if ( !DoorOpenNow && ( checktime - DoorClosedTime > DOORSWITCHDELAY ) && (DoorOnCount > 0) ) {
    Serial.println( "Door Count Reset");
    DoorOnCount = 0;
  }
#endif
}

void RingBell( int period) {
  Serial.println( "Bell");
  digitalWrite( CHAN2PIN, true);  // ON
  delay(period);
  digitalWrite( CHAN2PIN, false); // OFF
}

// Retransmit
char SetCmd[3] = "R1";

void sendCode( bool newstate) {
  Serial.println("Retransmit");

  if ( newstate ) {
#if REPEATENERGENIE == 1
    Serial.println("Send EG On");
    // Send ON
    digitalWrite( LEDPIN, LOW);
    const char *msg1on  = "0101000101010011101100001";
    sendcode( msg1on);
    digitalWrite( LEDPIN, HIGH);
    delay(INTERMSG);
#endif        
#if REPEATASK == 1
    digitalWrite( LEDPIN, LOW);
    SetCmd[0] = CHAN1COMMAND;
    SetCmd[1] = '1';
    Serial.print("Send ASK ");
    Serial.println(SetCmd);
    driver.send((uint8_t *)SetCmd, strlen(SetCmd));
    driver.waitPacketSent();
    delay(INTERMSG);
    driver.send((uint8_t *)SetCmd, strlen(SetCmd));
    driver.waitPacketSent();
    Serial.println("Sent");
    digitalWrite( LEDPIN, HIGH);
#endif        
  } else {
#if REPEATENERGENIE == 1
    Serial.println("Send EG Off");
    // Send OFF
    digitalWrite( LEDPIN, HIGH);
    const char *msg1off = "0101000101010011101100011";
    sendcode( msg1off);
    digitalWrite( LEDPIN, LOW);
    delay(INTERMSG);
#endif
#if REPEATASK == 1
    Serial.println("Send ASK -0");
    digitalWrite( LEDPIN, HIGH);
    SetCmd[0] = CHAN1COMMAND;
    SetCmd[1] = '0';
    driver.send((uint8_t *)SetCmd, strlen(SetCmd));
    driver.waitPacketSent();
    delay(INTERMSG);
    driver.send((uint8_t *)SetCmd, strlen(SetCmd));
    driver.waitPacketSent();
    Serial.println("Sent");
    digitalWrite( LEDPIN, LOW);
#endif        
  }
}
