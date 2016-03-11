// credits
// http://www.instructables.com/id/Arduino-Powered-Autonomous-Vehicle/

// V6.1
#define INC_MEMORY
#define INC_MOTOR
#define INC_COMPASS
#define INC_SD
//#define INC_SDREAD
//#define INC_GPS
#define INC_TINYGPS
//#define INC_NIETHER_GPS
#define INC_LCD
//#define INC_LCD_SPASH

#include <SPI.h> // shared - 0 bites dynamic
#include <SoftwareSerial.h> // shared 68 bytes dynamic

#ifdef INC_TINYGPS //////////////////////////////////////////////////
#include <TinyGPS.h>
#endif // __________________________________________________________

#ifdef INC_GPS /////////////////////////////////////////////////////
#include <Adafruit_GPS.h>  // gps - 0 BDi
#include <avr/sleep.h>  // 0 BD
#endif // __________________________________________________________

#ifdef INC_SD //////////////////////////////////////////////////////
//#include <SD.h> // gps 790 BD
#include <SdFat.h> // 18 DB
#endif // __________________________________________________________

#ifdef INC_MEMORY //////////////////////////////////////////////////
#include <SdFatUtil.h>
#endif // __________________________________________________________

#ifdef INC_COMPASS /////////////////////////////////////////////////
#include <Wire.h>  // compass & motor - 207 DB
#include <Adafruit_Sensor.h>  // compass - 0 DB
#include <Adafruit_HMC5883_U.h>  // compass - 0 DB
#endif // __________________________________________________________

#ifdef INC_MOTOR ///////////////////////////////////////////////////
//#include <Wire.h>  // compass & motor - 207 DB
#include <Adafruit_MotorShield.h> // motor - 0 DB
#include "utility/Adafruit_PWMServoDriver.h" //motor - 0 DB
#endif // __________________________________________________________

// shared
#define NUMBER_OF_WAYPOINTS 1
#define WAYPOINTTHRESHOLD_MEETERS 3

// jiffy lub parking lot left: 38.660809, -77.299244
// devels reach dead end 38.668115, -77.269902


//38.668283, -77.272059
//38.668006, -77.271996
//38.667880, -77.272019
//38.667805, -77.272055
//38.667729, -77.272118
//38.667619, -77.272344
//38.667511, -77.272569
//38.667480, -77.272776

const float waypoint_lats[NUMBER_OF_WAYPOINTS] = {38.668115};
const float waypoint_lons[NUMBER_OF_WAYPOINTS] = {-77.269902};
int waypoint_counter = 0;
float wayPointLatitudeDegrees = 38.861572;
float wayPointLongitudeDegrees = -77.218115;
int show_position_counter = 0;

#define HEADING_TOLERANCE 10
float distanceToWaypoint = -1;
float bearingToWaypoint;
float compassHeadingDegrees;
int waypoint_calculator;
int turn_direction;
#define DIRECTION_RIGHT 1
#define DIRECTION_LEFT -1
#define DIRECTION_STRAIGHT 0
#define chipSelect 10  // SD
const char delim = ',';
float latitudeDegrees = 38.6693; // aarons home by default
float longitudeDegrees = -77.2709;  // aarons home by default

#ifdef INC_SD //////////////////////////////////////////////////////
SdFat sd; // 57 DB
SdFile logfile_actual; // 565 DB;
SdFile* logfile = &logfile_actual;
#else
HardwareSerial* logfile = &Serial;
#endif // __________________________________________________________

#ifdef INC_TINYGPS //////////////////////////////////////////////////
TinyGPS gps;
SoftwareSerial ss(8, 7);
#define ledPin 13
long unsigned int age;
#endif // __________________________________________________________

#ifdef INC_GPS /////////////////////////////////////////////////////
SoftwareSerial mySerial(8, 7); // 28 DB
Adafruit_GPS GPS(&mySerial); // 325 DB
boolean usingInterrupt = false;
#define ledPin 13
// START CODE FROM http://jeffsinventions.com/autonomous-car-navigation/ ************
float latitudeRadians, wayPointLatitudeRadians, longitudeRadians, wayPointLongitudeRadians;
float deltaLatitudeRadians, deltaLongitudeRadians;
const float pi = 3.14159265;
const int radiusOfEarth = 6371; // in km
// END *******************************************************************************
#endif // __________________________________________________________

#ifdef INC_MOTOR
// START MOTOR ***********************
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // 68 DB
// Or, create it with a different I2C address (say for stacking)

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myPower = AFMS.getMotor(1); // 2 DB
Adafruit_DCMotor *mySteering = AFMS.getMotor(2); // 2 DB
#define MOTOR_POWER_SPEED 80
#define MOTOR_STEER_SPEED 255
// END MOTOR *************************
#endif // __________________________________________________________

#ifdef INC_LCD /////////////////////////////////////////////////////
SoftwareSerial lcd(2, 3); // 44 DB
SoftwareSerial* pout = &lcd;
#else
HardwareSerial* pout = &Serial;
#endif // __________________________________________________________

#ifdef INC_COMPASS /////////////////////////////////////////////////
/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345); // 52 DB
sensors_event_t compass_event;
#endif // _________________________________________________________


// **************************************************************************
// **************************************************************************
// **************************************************************************
// **************************************************************************
// **************************** START ***************************************
// **************************************************************************
// **************************************************************************
// **************************************************************************

// time,					satellites,	age,	speed,		angle,			latitudeDegrees,longitudeDegrees,	wayPointLatitudeDegrees,	wayPointLongitudeDegrees,	distanceToWaypoint(meeters),	distanceToWaypoint(miles)	bearingToWaypoint,	cardina		compass heading		turn direction	free mem

void setup() {
  pout_setup(115200); // baud rate ignored if LCD
  compass_setup();
  motor_setup();
  read_log_file();
  setup_sdfat();
  setup_gps();
  set_waypoints_from_counter();
#ifdef INC_GPS /////////////////////////////////////////////////////
  useInterrupt(true);
#endif // __________________________________________________________

}

void loop() {
  int gps_read = gps_parse_location();
  if (gps_read == 1) {
#ifdef INC_LCD
    show_position_counter++;
    if (show_position_counter > 20) {
      show_position_counter = 0;
      motor_stop();
      pout_clear();
      pout->print(latitudeDegrees, 6);
      pout_bottom();
      pout->print(longitudeDegrees, 6);
      delay(10000);
    }
#endif
    log_gps_data();
    caculate_waypoint();
    if (distanceToWaypoint < WAYPOINTTHRESHOLD_MEETERS) {
      motor_stop();
      pout_clear();
      pout_top();
      pout->print(F("WAYPOINT!!!!!"));
      pout_bottom();
      pout->print(distanceToWaypoint);
      pout->print(F(" < "));
      pout->print(WAYPOINTTHRESHOLD_MEETERS);
      pout->print(F(" meeters"));

      delay(3000);
      waypoint_counter++;
      if (waypoint_counter >= (NUMBER_OF_WAYPOINTS - 1)) {
        motor_stop();
        pout_clear();
        pout->print(F("ALL DONE!!!!"));
        delay (10000);
        while (1); // loop for ever
      } else {
        set_waypoints_from_counter();
        caculate_waypoint();
      }
    }
    compass_read_heading();
    sd_log_waypoint_info();
    set_direction();

    // LCD------------------------------------------
    pout_top();
    pout->print(show_position_counter);
    pout->write('B');
    pout->print((int)bearingToWaypoint);
    pout->write('H');
    pout->print((int)compassHeadingDegrees);
#ifdef INC_TINYGPS
    pout->write('G');
    pout->print((int)gps.f_course());
#endif
    pout_bottom();
#ifdef INC_TINYGPS
    pout->write('S');
    pout->print(gps.satellites());
#endif
    pout->write('M');
    pout->print((int)distanceToWaypoint);
    pout->write('T');
    pout->print(turn_direction);
    pout->write('F');
    pout->println(FreeRam());
    // END LCD------------------------------------------

    // compass heading, turn direction
    logfile->print(compassHeadingDegrees, 0);
    logfile->write(delim);
    logfile->print(turn_direction);
    logfile->write(delim);
#ifdef INC_MEMORY
    logfile->print(FreeRam());
#endif
    logfile->println();
    logfile->flush();
    motor_forward();
    motor_turn(turn_direction, 500);
  } else if (gps_read == -777) {
    pout_bottom();
    pout->print(F("GPS Not enabled"));
#ifdef INC_MEMORY
    pout_bottom();
    pout->print(F("Free RAM: "));
    pout->println(FreeRam());
#endif
    motor_stop();
    delay(3000);
  } else if (gps_read == 0) {
    pout_top();
    pout->print(F("wtng4fix h"));
    compass_read_heading(); // not needed but usefull for testing compass while sats are loading
    pout->print((int)compassHeadingDegrees);
//    delay(1000);
//    motor_turn(1,1000);    
//    delay(1000);
//    motor_turn(-1,1000); 
//    motor_forward();   
//    delay(1000);
//    motor_stop();    
#ifdef INC_MEMORY
    pout_bottom();
    pout->print(F("Free RAM: "));
    pout->println(FreeRam());    
    
    
#endif
    logfile->print(F("NOFIX"));
    logfile->println();
    logfile->flush();
    motor_stop();
    delay(3000);
  }
#ifdef INC_TINYGPS //////////////////////////////////////////////////
  smartdelay(2000);
#endif
}

// **************************************************************************
// **************************************************************************
// **************************************************************************
// **************************************************************************
// ****************************** END  **************************************
// **************************************************************************
// **************************************************************************
// **************************************************************************


void set_waypoints_from_counter() {
  wayPointLatitudeDegrees = waypoint_lats[waypoint_counter];
  wayPointLongitudeDegrees = waypoint_lons[waypoint_counter];
}


#ifdef INC_TINYGPS /////////////////////////////////////////////////////
void setup_gps() {
  pinMode(ledPin, OUTPUT); // not sure if this is needed for tynygps
  ss.begin(9600);
}


static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

int gps_parse_location() {

  gps.f_get_position(&latitudeDegrees, &longitudeDegrees, &age);

  // if a sentence is received, we can check the checksum, parse it...
  if (latitudeDegrees == TinyGPS::GPS_INVALID_F_ANGLE) {
    return 0;
  } else {
    return 1;
  }
}

void log_gps_data() {
  //  time,satellites,age,speed,angle,latitudeDegrees,longitudeDegrees

  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE) {
    logfile->print(F("INVALIDAGE"));
  } else {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d ", month, day, year, hour, minute, second);
    logfile->print(sz);
  }
  logfile->write(delim);
  logfile->print(gps.satellites(), 6);
  logfile->write(delim);
  logfile->print(age, 6);
  logfile->write(delim);
  logfile->print(gps.f_speed_mph(), 6);
  logfile->write(delim);
  logfile->print(gps.f_course(), 0);
  logfile->write(delim);
  logfile->print(latitudeDegrees, 6);
  logfile->print(delim);
  logfile->print(longitudeDegrees, 6);
  logfile->write(delim);
}

void caculate_waypoint() {
  distanceToWaypoint = TinyGPS::distance_between(latitudeDegrees, longitudeDegrees, wayPointLatitudeDegrees, wayPointLongitudeDegrees); // convert to miles
  // is this the best use of memory / compute power?  are we calculating 2x?
  bearingToWaypoint = TinyGPS::course_to(latitudeDegrees, longitudeDegrees, wayPointLatitudeDegrees, wayPointLongitudeDegrees);

}

void sd_log_waypoint_info() {
  // wayPointLatitudeDegrees,wayPointLongitudeDegrees,distanceToWaypointM,distanceMiles,bearingToWaypoint,cardinal
  logfile->print(wayPointLatitudeDegrees, 6);
  logfile->write(delim);
  logfile->print(wayPointLongitudeDegrees, 6);
  logfile->write(delim);
  logfile->print(distanceToWaypoint); //meters
  logfile->write(delim);
  logfile->print(distanceToWaypoint * 0.000621, 1); //miles
  logfile->write(delim);
  logfile->print(bearingToWaypoint, 0);
  logfile->write(delim);
  logfile->print(TinyGPS::cardinal(bearingToWaypoint));
  logfile->write(delim);
}

#endif // ________________________________________________________


#ifdef INC_NIETHER_GPS /////////////////////////////////////////////////////
void setup_gps() {}
void log_gps_data() {}
void useInterrupt(boolean v) {}
int gps_parse_location() {
  return -777;
}
void sd_log_waypoint_info() {}
void caculate_waypoint() {}
#endif // ________________________________________________________

#ifdef INC_GPS /////////////////////////////////////////////////////
void setup_gps() {
  pinMode(ledPin, OUTPUT);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_NOANTENNA);
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  }
  else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}


int gps_parse_location() {
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    char *stringptr = GPS.lastNMEA();
    if (!GPS.parse(stringptr)) {
      return -666; // parse falure.
    } else {
      if (!GPS.fix) {
        return 0;
      } else {
        latitudeDegrees = GPS.latitudeDegrees;
        longitudeDegrees = GPS.longitudeDegrees;
        return 1;
      }
    }
  } else {
    return -1;
  }
  return 0;
}


void log_gps_data() {
  //  year,month,day,hour,minute,seconds,speed,angle,satellites,GPS.latitude,longitude,latitudeDegrees,longitudeDegrees,fix,fixquality
  logfile->print(GPS.year);
  logfile->write(delim);
  logfile->print(GPS.month);
  logfile->write(delim);
  logfile->print(GPS.day);
  logfile->write(delim);
  logfile->print(GPS.hour);
  logfile->write(delim);
  logfile->print(GPS.minute);
  logfile->write(delim);
  logfile->print(GPS.seconds);
  logfile->write(delim);
  logfile->print(GPS.speed);
  logfile->write(delim);
  logfile->print(GPS.angle);
  logfile->write(delim);
  logfile->print(GPS.satellites);
  logfile->write(delim);
  logfile->print(GPS.latitude);
  logfile->print(GPS.lat);
  logfile->print(delim);
  logfile->print(GPS.longitude);
  logfile->print(GPS.lon);
  logfile->write(delim);
  logfile->print(GPS.latitudeDegrees);
  logfile->write(delim);
  logfile->print(GPS.longitudeDegrees);
  logfile->write(delim);
  logfile->print(GPS.fix);
  logfile->write(delim);
  logfile->print(GPS.fixquality);
  logfile->write(delim);
}



// START CODE FROM http://jeffsinventions.com/autonomous-car-navigation/ ***********************

void caculate_waypoint() {
  radianConversion();
  calculateBearing();
  calculateDistance();
}

// convert degrees to radians
void radianConversion() {
  deltaLatitudeRadians = (wayPointLatitudeDegrees - latitudeDegrees) * pi / 180;
  deltaLongitudeRadians = (wayPointLongitudeDegrees - longitudeDegrees) * pi / 180;
  latitudeRadians = latitudeDegrees * pi / 180;
  wayPointLatitudeRadians = wayPointLatitudeDegrees * pi / 180;
  longitudeRadians = longitudeDegrees * pi / 180;
  wayPointLongitudeRadians = wayPointLongitudeDegrees * pi / 180;
}

// calculate distance from present location to next way point
void calculateDistance() {
  float a = sin(deltaLatitudeRadians / 2) * sin(deltaLatitudeRadians / 2) + sin(deltaLongitudeRadians / 2) * sin(deltaLongitudeRadians / 2) * cos(latitudeRadians) * cos(wayPointLatitudeRadians);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  distanceToWaypoint = radiusOfEarth * c; // distance in kilometers
  //distanceToWaypoint = distanceToWaypoint * 0.621371192; // distance in miles
}

// calculate bearing from present location to next way point
void calculateBearing() {
  float y = sin(deltaLongitudeRadians) * cos(wayPointLatitudeRadians);
  float x = cos(latitudeRadians) * sin(wayPointLatitudeRadians) - sin(latitudeRadians) * cos(wayPointLatitudeRadians) * cos(deltaLongitudeRadians);
  bearingToWaypoint = atan2(y, x) / pi * 180;

  if (bearingToWaypoint < 0) {
    bearingToWaypoint = 360 + bearingToWaypoint;
  }
}

void sd_log_waypoint_info() {
  // wayPointLatitudeDegrees,wayPointLongitudeDegrees,distanceToWaypoint,bearingToWaypoint
  logfile->print(wayPointLatitudeDegrees);
  logfile->write(delim);
  logfile->print(wayPointLongitudeDegrees);
  logfile->write(delim);
  logfile->print(distanceToWaypoint);
  logfile->write(delim);
  logfile->print(bearingToWaypoint);
  logfile->write(delim);
}

#endif // ________________________________________________________



#ifdef INC_COMPASS //////////////////////////////////////////////////
// https://learn.adafruit.com/adafruit-hmc5883l-breakout-triple-axis-magnetometer-compass-sensor/
void compass_setup() {
  if (!compass.begin()) {
    /* There was a problem detecting the HMC5883 ... check your connections */
    // TODO this doesn't fail?! watch out don't rely on this message
    pout_top();
    pout->print(F("No compass !!!"));
    delay (5000);
    while (1);
  }
}

void compass_read_heading(void) {
  compass.getEvent(&compass_event);
  float heading = atan2(compass_event.magnetic.y, compass_event.magnetic.x);

  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Cedar Park, TX: Magnetic declination: 4Â° 11' EAST (POSITIVE);  1 degreee = 0.0174532925 radians

#define DEC_ANGLE 0.069
  heading += DEC_ANGLE;

  // Correct for when signs are reversed.
  if (heading < 0)
    heading += 2 * PI;

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI)
    heading -= 2 * PI;

  // Convert radians to degrees for readability.
  compassHeadingDegrees = heading * 180 / M_PI;
}


#else
void compass_setup() {}
void compass_read_heading() {
  pout_top();
  pout->print(F("Fake Data"));
  delay(500);
  compassHeadingDegrees = 180;
}
#endif // ________________________________________________________

void set_direction() {
  // calculate where we need to turn to head to destination
  int headingError = bearingToWaypoint - compassHeadingDegrees;

  // adjust for compass wrap
  if (headingError < -180) {
    headingError += 360;
  }
  if (headingError > 180) {
    headingError -= 360;
  }

  // calculate which way to turn to intercept the targetHeading
  if (abs(headingError) <= HEADING_TOLERANCE) {     // if within tolerance, don't turn
    turn_direction = DIRECTION_STRAIGHT;
  } else if (headingError < 0) {
    turn_direction = DIRECTION_LEFT;
  } else if (headingError > 0) {
    turn_direction = DIRECTION_RIGHT;
  } else {
    turn_direction = DIRECTION_STRAIGHT;
  }
}


// START POUT V1.2 ************************
// https://www.sparkfun.com/tutorials/246

void pout_clear() {
  pout_bottom(true);
  pout_top(true); // so we end at top
}

void pout_top() {
  pout_top(true);
}
void pout_top(boolean clear_contents) {
  pout_set_cursor(true, clear_contents);
}
void pout_bottom() {
  pout_bottom(true);
}
void pout_bottom(boolean clear_contents) {
  pout_set_cursor(false, clear_contents);
}

#ifdef INC_LCD //////////////////////////////////////////////////

// in this block you can use lcd.print or pout->print... same thing

void pout_set_cursor(boolean is_top_string, boolean clear_contents) {
  if (clear_contents) {
    // clear
    lcd.write(254); // special char
    lcd.write(is_top_string ? 128 : 192); // first line = 128 second line = 192
    for (int i = 0; i < 16; i++) {
      lcd.write(' ');
    }
  }
  // write string
  lcd.write(254); // special char
  lcd.write(is_top_string ? 128 : 192); // first line = 128 second line = 192
}

void pout_setup(unsigned long baud_rate) {
  lcd.begin(9600);  // Start the LCD at 9600 baud
  delay(1000);
  pout_clear();

#ifdef INC_LCD_SPASH
  // test
  for (int i = 0; i < 32; i++) {
    pout->write('*');
  }
  delay(1000);

  // test 2 overwrite * chars
  pout_top(false);
  pout->print(F("POUT"));
  pout_bottom(false);
  pout->print(F("started!"));
  delay(1000);

  // test 3 clear trailing *
  pout_top();
  pout->print(F("POUT"));
  pout_bottom();
  pout->print(F("started!"));
  delay(1000);
#else
  pout->print(F("pout started"));
  delay(1000);
#endif
}

#else

void pout_set_cursor(boolean is_top_string, boolean clear_contents) {}
void pout_setup(unsigned long baud_rate) {
  Serial.begin(baud_rate);
  delay(500);
  pout->println(F("POUT... started"));
}

#endif // ________________________________________________________


#ifdef INC_MOTOR //////////////////////////////////////////////////
void motor_setup() {
  AFMS.begin();  // create with the default frequency 1.6KHz
  myPower->setSpeed(MOTOR_POWER_SPEED);
  myPower->run(RELEASE);// prob not needed
  mySteering->setSpeed(MOTOR_STEER_SPEED);
  mySteering->run(RELEASE); // prob not needed
}

void motor_forward() {
  myPower->setSpeed(MOTOR_POWER_SPEED);
  myPower->run(FORWARD);
}

void motor_stop() {
  //myPower->setSpeed(0);
  myPower->run(RELEASE);
}

void motor_turn(int which_way, int how_long) {
  if (which_way != DIRECTION_STRAIGHT) {
    if (which_way == DIRECTION_RIGHT) {
      mySteering->run(FORWARD);
    } else {
      mySteering->run(BACKWARD);
    }
    delay(how_long);
  }
  mySteering->run(RELEASE);
}

#else
void motor_turn(int which_way, int how_long) {}
void motor_setup() {}
void motor_forward() {}
void motor_stop() {}
#endif // ________________________________________________________


#ifdef INC_SD ///////////////////////////////////////////////////////
// SDFat *****************************
// https://github.com/greiman/SdFat
// look at read/write example for more info
// note there is a smaller lib for fat16 < 2GB
void setup_sdfat() {
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) sd.initErrorHalt();
  if (!logfile->open("L1.CSV", O_RDWR | O_CREAT | O_AT_END)) {
    sd.errorHalt("SD!");
  }
}
#else
void setup_sdfat() {
Serial.begin(115290);
}
#endif // ________________________________________________________

#ifdef INC_SDREAD///////////////////////////////////////////////////
// doesnt work after setup if the file has been opened!
void read_log_file() {
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) sd.initErrorHalt();
  // re-open the file for reading:
  if (!logfile->open("L1.CSV", O_READ)) {
    sd.errorHalt("SD!");
  }

  // read from the file until there's nothing else in it:
  int data;
  while ((data = logfile->read()) >= 0) Serial.write(data);
  logfile->close();
}
#else
void read_log_file() {}
#endif // __________________________________________________________




