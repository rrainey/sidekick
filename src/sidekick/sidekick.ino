#include <Adafruit_BME680.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#include <SPI.h>
#include <SD.h>

#define APP_STRING  "Sidekick, version 0.1"
#define LOG_VERSION 1
#define NMEA_APP_STRING "$PVER,\"Sidekick, version 0.1\",1"

/*
 * Operating mode
 */
#define OPS_FLIGHT       0 // normal mode (NOT YET IMPLEMENTED)
#define OPS_GROUND_TEST  1 // for testing; uses horizontal movement as an analogue to altitude changes

int nOpMode = OPS_GROUND_TEST;

#define TEST_SPEED_THRESHOLD_KTS  6.0

/*
 * Automatic jump logging
 * Generate a unique log file for each jump.
 * Log file contains GPS NMEA CSV records along with extra sensor data records in quasi-NMEA format
 * 
 * State 0: WAIT - gather baseline surface elevation information; compute HDOT_fps
 * 
 * State 1: IN_FLIGHT (enter this state when HDOT_fpm indicates >= 200 fpm climb), 
 *                   enable GPS (future versions), compute HDOT_fps, start logging if not already)
 *                   
 * State 2: LANDED1 (enter when altitude is within 1000 feet of baseline ground alt and 
 *                   HDOT_fpm < 50 fpm, start timer 1, log data)
 *                   
 * State 3: LANDED2  like state 2 - if any conditions are vioated, return to state 1(IN_FLIGHT), 
 *                   go to state 0 when timer 1 reaches 60 seconds, disable GPS (future versions), log data otherwise
 */

#define STATE_WAIT       0
#define STATE_IN_FLIGHT  1
#define STATE_LANDED_1   2
#define STATE_LANDED_2   3

int nAppState;

#define BLINK_STATE_OFF     0
#define BLINK_STATE_LOGGING 1
#define BLINK_STATE_BATTERY 2

int blinkState;

bool bBatteryAlarm = false;
float measuredBattery_volts;

/*
 * LiPoly battery is rated at 3.7V
 */
#define LOWBATT_THRESHOLD 3.55

/*
 * Estimated MSL altitude, based on standard day pressure @ sea level
 */
int nH_feet = 0;

/*
 * Estimated rate of climb (fps)
 */
int nHDOT_fps = 0;

/*
 * Estimated ground elevation, ft
 * 
 * Computed while in WAIT state.
 */
int nHGround_feet = 0;

/*
 * I2C connection to the BME688 pressure/temp sensor
 */
Adafruit_BME680 bme;

/*
 * I2C connection to the MPU-6050 IMU
 */
Adafruit_MPU6050 mpu;

bool mpu6050Present = false;

#define SEALEVELPRESSURE_HPA (1013.25)

/*
 * Adalogger M0 hardware definitions
 * 
 * See https://learn.adafruit.com/adafruit-feather-m0-adalogger/pinouts
 */
#define VBATPIN       A7
#define RED_LED       13
#define GREEN_SD_LED   8
#define SD_CHIP_SELECT 4

#define GPSSerial Serial1

File logFile;

char logpath[32];

Adafruit_GPS GPS(&GPSSerial);

/*
 * Records last millis() time when timers were updated in
 * the main loop.
 */
uint32_t lastTime_ms = 0;

#define TIMER1_INTERVAL_MS 60000

bool bTimer1Active = false;
uint32_t timer1_ms = 0;

#define TIMER2_INTERVAL_MS 30000

bool bTimer2Active = false;
uint32_t timer2_ms = 0;

#define TIMER3_ON_INTERVAL_MS     750
#define TIMER3_OFF_INTERVAL_1_MS  750 // off interval when signaling battery low
#define TIMER3_OFF_INTERVAL_2_MS  (3000 - TIMER3_ON_INTERVAL_MS) // off interval for flight mode

bool bTimer3Active = false;
uint32_t timer3_ms = 0;

#define TIMER4_INTERVAL_MS 200

bool bTimer4Active = false;
uint32_t timer4_ms = 0;

#define TIMER5_INTERVAL_MS 10000

bool bTimer5Active = false;
uint32_t timer5_ms = 0;

int redLEDState = LOW;

bool printNMEA = false;

void setBlinkState( int newState ) {

  if ( blinkState == BLINK_STATE_OFF ) {
    if (newState != BLINK_STATE_OFF ) {
      blinkState = newState;
      bTimer3Active = true;
      timer3_ms = TIMER3_ON_INTERVAL_MS;
      redLEDState = HIGH;
      digitalWrite(RED_LED, redLEDState);
    }
  }
  else if ( blinkState == BLINK_STATE_LOGGING ) {
    if (newState == BLINK_STATE_BATTERY) {
      blinkState = newState;
      bTimer3Active = true;
      timer3_ms = TIMER3_ON_INTERVAL_MS;
      redLEDState = HIGH;
      digitalWrite(RED_LED, redLEDState);
    }
    else if (newState == BLINK_STATE_OFF) {
        blinkState = newState;
        bTimer3Active = false;
        redLEDState = LOW;
        digitalWrite(RED_LED, redLEDState);
     }
  }
  else if ( blinkState == BLINK_STATE_BATTERY ) {
    if (newState == BLINK_STATE_OFF) {
      bTimer3Active = true;
      redLEDState = LOW;
      digitalWrite(RED_LED, redLEDState);
    }
    else {
      // change state, but let blinking logic handle the transition
      blinkState = newState;
    }
  }
}

char * generateLogname(char *gname) 
{
    char name[128];
    int i;
    for (i=0; true; i++) {
      sprintf (name, "log%05d.txt", i);
   
      if (!SD.exists(name)) {
          strcpy(gname, name);
          return gname;
      }
    }

    return NULL;
}

void setup() {

  blinkState = BLINK_STATE_OFF;

  lastTime_ms = millis();

  /*
   * Set up timers
   * 
   * Timer 1: used in landing state machine
   * 
   * Timer 2: periodic check of battery state
   * 
   * Timer 3: blink controller for RED LED
   * 
   * Timer 4: BME sensor logging interval timer
   */
  
  bTimer2Active = true;
  timer2_ms = TIMER2_INTERVAL_MS;

  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);

  // wait for hardware serial to appear
  while (!Serial);
  
  Serial.begin(115200);

  Serial.println(APP_STRING);
  
  // 9600 baud is the default rate for the GPS
  GPSSerial.begin(9600);

  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // For test operating mode, set update rate to 1HZ
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  // include antenna status in stream
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  if (!SD.begin( SD_CHIP_SELECT )) {

    Serial.println("SD card initialization failed!");

    while (1);

  }

  Serial.println("Adafruit MPU6050 test");

  if (mpu.begin()) {
    Serial.println("MPU6050 present");

    mpu6050Present = true;

    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  else {
    Serial.println("Failed to find MPU6050 chip");
    mpu6050Present = false;
  }

  
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  //bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  //bme.setGasHeater(320, 150); // 320*C for 150 ms

  Serial.println("Switching to STATE_WAIT");
  nAppState = STATE_WAIT;
  
}


void loop() {
  
  uint32_t curTime_ms = millis();

  uint32_t deltaTime_ms = curTime_ms - lastTime_ms;

  if ( deltaTime_ms > 0 ) {

    /*
     * Update active timer countdowns
     */
    if (bTimer1Active) {
      timer1_ms -= deltaTime_ms;
    }
  
    if (bTimer2Active) {
      timer2_ms -= deltaTime_ms;
    }
  
    if (bTimer3Active) {
      timer3_ms -= deltaTime_ms;
    }
  
    if (bTimer4Active) {
      timer4_ms -= deltaTime_ms;
    }

    if (bTimer5Active) {
      timer5_ms -= deltaTime_ms;
    }

  }
  
  char c = GPS.read();

  /*
   * Received a NMEA record terminator?  Process it.
   */
  if ( GPS.newNMEAreceived() ) {

    char lastNMEA[MAXLINELENGTH];

    strcpy( lastNMEA, GPS.lastNMEA() );

    if ( printNMEA ) {
      Serial.print( lastNMEA );
    }

    /*
     * Parse this latest arriving NMEA record. This will update appropriate state
     * variables in the GPS object. Calling GPS.lastNMEA() also has the effect of clearing the
     * flag indicating a new record arrived.
     */
    if (!GPS.parse( lastNMEA )) {
      // message not useful to us, or (less likely) had invalid checksum
    }

    /**
     * State machine appropriate for ground testing
     * TODO: support flight operating mode, OPS_FLIGHT
     */

    switch (nAppState) {

    case STATE_WAIT:
      if (GPS.speed >= TEST_SPEED_THRESHOLD_KTS) {

        Serial.println("Switching to STATE_IN_FLIGHT");
        
        // open log file
        generateLogname( logpath );
        logFile = SD.open( logpath, FILE_WRITE );

        logFile.println( NMEA_APP_STRING );
        
        logFile.print( lastNMEA );

        logFile.flush();
 
        // log data; jump to 5HZ logging
        //GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
        //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);

        // Activate altitude / battery sensor logging
        bTimer4Active = true;
        timer4_ms = TIMER4_INTERVAL_MS;

        // Activate periodic log file flushing
        bTimer5Active = true;
        timer5_ms = TIMER5_INTERVAL_MS;

        // Activate "in flight" LED blinking
        setBlinkState ( BLINK_STATE_LOGGING );
        
        nAppState = STATE_IN_FLIGHT;
      }
      break;

    case STATE_IN_FLIGHT:
      {
        // log NMEA string
        logFile.print( lastNMEA );

        flushLog();

        if (GPS.speed < TEST_SPEED_THRESHOLD_KTS) {
          Serial.println("Switching to STATE_LANDED_1");
          nAppState = STATE_LANDED_1;
          timer1_ms = TIMER1_INTERVAL_MS;
        }
      }
      break;

    case STATE_LANDED_1:
      {
        // log NMEA string
        logFile.print( lastNMEA );

        flushLog();

        if (GPS.speed >= TEST_SPEED_THRESHOLD_KTS) {
          Serial.println("Switching to STATE_IN_FLIGHT");
          nAppState = STATE_IN_FLIGHT;
          bTimer1Active = false;
        }
        else if (bTimer1Active && timer1_ms <= 0) {
          GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
          GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
          bTimer4Active = false;
          logFile.close();
          Serial.println("Switching to STATE_WAIT");
          setBlinkState ( BLINK_STATE_OFF );
          nAppState = STATE_WAIT;
          bTimer1Active = false;
          bTimer5Active = false;
          
        }
        
      }
      break;

    case STATE_LANDED_2:
      {
        // log NMEA string
        logFile.print( lastNMEA );

        flushLog();
        
        if (GPS.speed >= 7.0) {
          nAppState = STATE_IN_FLIGHT;
          bTimer1Active = false;
        }
        else if (bTimer1Active && timer1_ms <= 0) {
          GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
          GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
          bTimer4Active = false;
          logFile.close();
          setBlinkState ( BLINK_STATE_OFF );
          nAppState = STATE_WAIT;
          Serial.println("Switching to STATE_WAIT");
          bTimer1Active = false;
          bTimer5Active = false;
        }
      }
      break;

    }
      
  }

  /*
   * Processing tasks below are outside of the
   * GPS NMEA processing loop.
   */

  /*
   * Every 30 seconds, measure the battery state.
   * Blink red LED if low.
   */
  if (bTimer2Active && timer2_ms <= 0) {
    
    timer2_ms = TIMER2_INTERVAL_MS;
    
    float measuredBattery_volts = analogRead(VBATPIN);
    measuredBattery_volts *= 2;    // we divided by 2, so multiply back
    measuredBattery_volts *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredBattery_volts /= 1024; // convert to voltage

    if ( measuredBattery_volts <= LOWBATT_THRESHOLD ) {
      bBatteryAlarm = true;
      setBlinkState ( BLINK_STATE_BATTERY );
    }
    else {
      if ( bBatteryAlarm ) {
        setBlinkState( (nAppState != STATE_WAIT) ? BLINK_STATE_LOGGING : BLINK_STATE_OFF );
      }
      bBatteryAlarm = false;
    }
  }

  /*
   * RED LED Blink Logic
   * 
   * The RED LED will blink using different patterns to indicate
   * one of three states: Constant off, ON/OFF at 1.5Hz to indicates a low battery.
   * A 3-second blink is used to indicate flight mode.
   */
  if (bTimer3Active && timer3_ms <= 0) {
    redLEDState = (redLEDState == HIGH) ? LOW: HIGH;
    digitalWrite(RED_LED, redLEDState);
    timer3_ms = TIMER3_OFF_INTERVAL_1_MS;
    if (!bBatteryAlarm && redLEDState == LOW) {
      timer3_ms = TIMER3_OFF_INTERVAL_2_MS;
    }
    
  }

  /*
   * Log BME sensor information
   */
  if (bTimer4Active && timer4_ms <= 0) {
 
    BMESample();

    timer4_ms = TIMER4_INTERVAL_MS;
  }

  if ( deltaTime_ms > 0 ) {

    lastTime_ms = curTime_ms;

  }
  
}

void BMESample() {

  if (nAppState != STATE_WAIT) {
    
    if (! bme.performReading()) {
      Serial.println("Failed to perform BME888 reading :(");
      return;
    }

    if (logFile ) {
    
      logFile.print("$PENV,");
      logFile.print(bme.temperature);
      logFile.print(",");
      logFile.print(bme.pressure / 100.0);
      logFile.print(",");
      logFile.print(bme.humidity);
      logFile.println(",");
      logFile.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
      logFile.print(",");
      logFile.print(bme.gas_resistance / 1000.0);
      logFile.print(",");
      logFile.println(measuredBattery_volts);

    }
  }
}

void flushLog() {
  
  if ( nAppState != STATE_WAIT ) {
    
    if (bTimer5Active && timer5_ms <= 0) {

      if ( logFile ) {
        logFile.flush();
      }

      timer5_ms = TIMER5_INTERVAL_MS;
    }
  }
}
