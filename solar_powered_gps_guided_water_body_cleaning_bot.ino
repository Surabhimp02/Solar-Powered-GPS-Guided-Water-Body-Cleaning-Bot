#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <HMC5883L.h>


int number_of_points = 2;


int z = 0;
int GPS_Course;
int Number_of_SATS;
TinyGPSPlus gps;

int16_t mx, my, mz;
int desired_heading;
int compass_heading;
int compass_dev = 5;


int Heading_A;
int Heading_B;
int pass = 0;
unsigned long Distance_To_Home;

int ac = 0;
int wpCount = 0;
double Home_LATarray[50];
double Home_LONarray[50];


int increment = 0;

SoftwareSerial serial2(10, 11);
const int HighL =4;  // LEFT SIDE MOTOR
const int LowL = 5;

HMC5883L compass;
const int HighR = 6;  //RIGHT SIDE MOTOR
const int LowR = 7;
float headingDegrees = 0;
//***8
int minX = 0;
int maxX = 0;
int minY = 0;
int maxY = 0;
int minZ = 0;
int maxZ = 0;
int offX = 0;
int offY = 0;
int offZ = 0;
int caliberror = 0;
unsigned long pt = 0;
void setup() {
  // pinMode(buz, OUTPUT);
  // digitalWrite(buz, LOW);
  serial2.begin(9600);
  Serial.begin(9600);
  pinMode(HighL, OUTPUT);
  pinMode(LowL, OUTPUT);
  // pinMode(8, OUTPUT);
  // pinMode(D5, OUTPUT);
  // pinMode(D6, OUTPUT);
  pinMode(HighR, OUTPUT);
  pinMode(LowR, OUTPUT);
  // pinMode(echoPin1, INPUT);

  // pinMode(pingPin1, OUTPUT);
  Serial.println("Initialize HMC5883L");
  while (!compass.begin()) {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }
  Serial.println("1");
  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino



  Serial.println("2");
  Startup();
}







void loop() {
  Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = 0;
  heading += declinationAngle;
  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0) {
    heading += 2 * PI;
  }

  if (heading > 2 * PI) {
    heading -= 2 * PI;
  }

  // Convert to degrees
  headingDegrees = heading * 180 / M_PI;
  int z = headingDegrees;
  z = 360 - z;
  z = z - 360 + caliberror;
  if (z > 0) {
    if (z > 360 && z < 720) {
      z = z - 360;
    }
    if (z > 720 && z < 1080) {
      z = z - 720;
    }
    if (z > 1080 && z < 1440) {
      z = z - 1080;
    }
    if (z > 1440 && z < 1800) {
      z = z - 1440;
    }
    if (z > 1800 && z < 2160) {
      z = z - 1800;
    }
    if (z > 2160 && z < 2520) {
      z = z - 2160;
    }
  } else {
    if (z < -360 && z > -720) {
      z = z + 360;
    }
    if (z < -720 && z > -1080) {
      z = z + 720;
    }
    if (z < -1080 && z > -1440) {
      z = z + 1080;
    }
    if (z < -1440 && z > -1800) {
      z = z + 1440;
    }
    if (z < -1800 && z > -2160) {
      z = z + 1800;
    }
    if (z < -2160 && z > -2520) {
      z = z + 2160;
    }
    z = 360 + z;
  }


  headingDegrees = z;
 
  car();
}
void car() {
  gpsInfo();
  if (ac < number_of_points) {  // Start of Go_Home procedure
                                //(1000);

    //Serial.println("on way");
    // Update Compass heading
    getGPS();  // Tiny GPS function that retrieves GPS data - update GPS location// delay time changed from 100 to 10



    Distance_To_Home = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), Home_LATarray[ac], Home_LONarray[ac]);  //Query Tiny GPS for Distance to Destination
    GPS_Course = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), Home_LATarray[ac], Home_LONarray[ac]);               //Query Tiny GPS for Course to Destination
                                                                                                                                    //GPS_Course=90;

    if (Distance_To_Home < 10)  // If the Vehicle has reached it's Destination, then Stop
    {
      stopboat();  // Stop the robot after each waypoint is reached
                   // Serial.println("arrived!");                    // Print to Bluetooth device - "You have arrived"
      ac++;        // increment counter for next waypoint
                   // Break from Go_Home procedure and send control back to the Void Loop
      // go to next waypoint
    }


    if (abs(GPS_Course - headingDegrees) <=6)  // If GPS Course and the Compass Heading are within x degrees of each other then go Forward
    // otherwise find the shortest turn radius and turn left or right
    {
      Forward();
      // analogWrite(D5, 255);
      // analogWrite(D6, 255);  // Go Forward
      Serial.println("f");

    } else {

      if ((GPS_Course - headingDegrees) >= -2)  // if z is less than 180 and not a negative value then turn left otherwise turn right
      {

        if (GPS_Course - headingDegrees < 10) {
          Serial.println("sr");
         SlowRightTurn();
          // analogWrite(D5, 130);  // pin 9 is pwm / power pin of right motor   0-255
          // analogWrite(D6, 255);  // pin 3 is pwm / power pin of left  motor

        } else {
          Serial.println("fr");
          SlowRightTurn();
          // analogWrite(D5, 255);
          // analogWrite(D6, 255);
        }



      } else {

        if ((GPS_Course - headingDegrees) > -10) {
           
          SlowLeftTurn();
          // analogWrite(D5, 255);
          // analogWrite(D6, 130);
          Serial.println("sl");
        } else {
          SlowLeftTurn();
          // analogWrite(D5, 255);
          // analogWrite(D6, 255);
          Serial.println("fl");
        }
      }
    }
    Serial.print(headingDegrees);
    Serial.print(",");
    Serial.println(GPS_Course);



  } else {
    stopboat();
  }
}








void Startup() {


  // while (Number_of_SATS <= 3)                         // Wait until x number of satellites are acquired before starting main loop
  // {
  //   //Serial.println("3   ");
  //   getGPS();                                         // Update gps data
  //   Number_of_SATS = (int)(gps.satellites.value());   // Query Tiny GPS for the number of Satellites Acquired

  //                                    // Check to see if there are any bluetooth commands being received
  // }
  // setWaypoint();                                      // set intial waypoint to current location
  Serial.println("3");
  Serial.println("press 1 to calibrae ,press 2 if done");
  int c = takeip().toInt();
  if (c == 1) {
    while (true) {
      Vector mag = compass.readRaw();

      // Determine Min / Max values
      if (mag.XAxis < minX) minX = mag.XAxis;
      if (mag.XAxis > maxX) maxX = mag.XAxis;
      if (mag.YAxis < minY) minY = mag.YAxis;
      if (mag.YAxis > maxY) maxY = mag.YAxis;
      if (mag.ZAxis < minZ) minZ = mag.ZAxis;
      if (mag.ZAxis > maxZ) maxZ = mag.ZAxis;

      // Calculate offsets
      offX = (maxX + minX) / 2;
      offY = (maxY + minY) / 2;
      offZ = (maxZ + minZ) / 2;
      Vector norm = compass.readNormalize();
      float heading = atan2(norm.YAxis, norm.XAxis);

      // Set declination angle on your location and fix heading
      // You can find your declination on: http://magnetic-declination.com/
      // (+) Positive or (-) for negative
      // For Bytom / Poland declination angle is 4'26E (positive)
      // Formula: (deg + (min / 60.0)) / (180 / M_PI);
      float declinationAngle = 0;
      heading += declinationAngle;
      // Correct for heading < 0deg and heading > 360deg
      if (heading < 0) {
        heading += 2 * PI;
      }

      if (heading > 2 * PI) {
        heading -= 2 * PI;
      }

      // Convert to degrees
      headingDegrees = heading * 180 / M_PI;
      if (millis() - pt > 1500) {
        Serial.print("xoff: ");
        Serial.println(offX);
        Serial.print("yoff: ");
        Serial.println(offY);
        Serial.print("zoff: ");
        Serial.println(offZ);
        Serial.print("headng");
        Serial.println(headingDegrees);
        Serial.println("........");
        pt = millis();
      }
    }
  }
  Serial.println("enter xoffset");
  offX = takeip().toInt();
  Serial.println("enter yoffset");
  offY = takeip().toInt();
  Serial.println("enter Zoffset");
  offZ = takeip().toInt();
  //compass.setOffset(-6,290,-70);
  compass.setOffset(offX, offY, offZ);
  for (int k = 0; k < 10; k++) {
    Vector norm = compass.readNormalize();
    float heading = atan2(norm.YAxis, norm.XAxis);
    float declinationAngle = 0;
    heading += declinationAngle;
    // Correct for heading < 0deg and heading > 360deg
    if (heading < 0) {
      heading += 2 * PI;
    }

    if (heading > 2 * PI) {
      heading -= 2 * PI;
    }

    // Convert to degrees
    headingDegrees = heading * 180 / M_PI;


    Serial.println("heading");
    Serial.println(headingDegrees);
    delay(100);
  }
  Serial.println("enter calib Error");
  caliberror = takeip().toInt();
  for (int l = 0; l < 10; l++) {
    Vector norm = compass.readNormalize();
    float heading = atan2(norm.YAxis, norm.XAxis);
    float declinationAngle = 0;
    heading += declinationAngle;
    // Correct for heading < 0deg and heading > 360deg
    if (heading < 0) {
      heading += 2 * PI;
    }

    if (heading > 2 * PI) {
      heading -= 2 * PI;
    }

    // Convert to degrees
    headingDegrees = heading * 180 / M_PI;
    int z = headingDegrees;
    z = 360 - z;
    z = z - 360 + caliberror;
    if (z > 0) {
      if (z > 360 && z < 720) {
        z = z - 360;
      }
      if (z > 720 && z < 1080) {
        z = z - 720;
      }
      if (z > 1080 && z < 1440) {
        z = z - 1080;
      }
      if (z > 1440 && z < 1800) {
        z = z - 1440;
      }
      if (z > 1800 && z < 2160) {
        z = z - 1800;
      }
      if (z > 2160 && z < 2520) {
        z = z - 2160;
      }
    } else {
      if (z < -360 && z > -720) {
        z = z + 360;
      }
      if (z < -720 && z > -1080) {
        z = z + 720;
      }
      if (z < -1080 && z > -1440) {
        z = z + 1080;
      }
      if (z < -1440 && z > -1800) {
        z = z + 1440;
      }
      if (z < -1800 && z > -2160) {
        z = z + 1800;
      }
      if (z < -2160 && z > -2520) {
        z = z + 2160;
      }
      z = 360 + z;
    }


    headingDegrees = z;



    Serial.println("heading");
    Serial.println(headingDegrees);
    delay(100);
  }
  Serial.println("enter number of way points");
  number_of_points = takeip().toInt();
  Serial.println("number of waypoints entered");
  Serial.println(number_of_points);
  for (int l = 0; l < number_of_points; l++) {

    Serial.print("enter lat of ");
    Serial.print(l);
    Serial.print(" waypoint");
    Home_LATarray[l] = atof(takeipcordinate().c_str());
    Serial.println(Home_LATarray[0], 7);
    Serial.print("enter long of ");
    Serial.print(l);
    Serial.print(" waypoint");
    Home_LONarray[l] = atof(takeipcordinate().c_str());
    Serial.println(Home_LATarray[0], 7);
  }



  ac = 0;  // zero array counter

  //   Serial.print(Number_of_SATS);
  //   Serial.print(" Satellites Acquired");
}
void getGPS()  // Get Latest GPS coordinates
{

  while (serial2.available() > 0)
    gps.encode(serial2.read());
}
void gpsInfo()  // displays Satellite data to user
{
  Number_of_SATS = (int)(gps.satellites.value());                                                                                 //Query Tiny GPS for the number of Satellites Acquired
  Distance_To_Home = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), Home_LATarray[ac], Home_LONarray[ac]);  //Query Tiny GPS for Distance to Destination

  //Serial.print("Distance to Home ");
  //  Serial.println(Distance_To_Home);
}
void Forward() {
  digitalWrite(HighL, HIGH);
  digitalWrite(LowL, LOW);
  digitalWrite(HighR,HIGH);
  digitalWrite(LowR, LOW);
  // Serial.println("forward ");
}
void SlowRightTurn() {
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  digitalWrite(HighR, HIGH);
  digitalWrite(LowR, LOW);
  // Serial.println("Right");
}
void SlowLeftTurn() {

  digitalWrite(HighL, HIGH);
  digitalWrite(LowL, LOW);
  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  // Serial.println("LEFT");
}
void stopboat() {
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, LOW);
  digitalWrite(HighR, LOW);
  digitalWrite(LowR, LOW);
  //Serial.println("STOP ");
}
void backward() {
  digitalWrite(HighL, HIGH);
  digitalWrite(LowL, LOW);
  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  // Serial.println("forward ");
}

long microsecondsToInches(long microseconds) {
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}
String takeip() {
  String ip = "";
  while (true) {
    if (Serial.available() > 0) {
      ip = Serial.readString();
      break;
    }
  }
  return ip;
}

String takeipcordinate() {
  String ip = "";
  while (true) {
    if (Serial.available() > 0) {
      ip = Serial.readString();
      break;
    }
  }
  return ip;
}
