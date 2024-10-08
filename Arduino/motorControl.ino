/*V2 This program reads strings from the serial monitor of the form
 * 
 *            AZ<integer string>EL<integer string>
 * 
 * where the two integer strings represent an azimuth and
 * elevation in whole degrees. The program parses these strings, 
 * called Azimuth and Elevation, respectively,
 * and converts the two integer strings into an azimuth integer,  
 * Az_input, and an elevation integer, El_input.
 * 
 * Az_input becomes the input to a linear function whose output
 * is a pulse width, Az, in microseconds. This pulse width is written
 * to the azimuth servo, Azservo, so that it assumes the azimuth angle
 * requested by the azimuth integer string. El_input is mapped into
 * a pulse width, El, by another linear function to govern the angle 
 * of the elevation servo, ElServo. 
*/
#include <Servo.h>
int Azpin = 9;
int Elpin = 6;
Servo AzServo;
Servo ElServo;
int Az = 1506;//1506
// new initial El value for “factory fresh” Stingray-9’s
int El = 1498;//1498
int Az_input;
int El_input;

String Azimuth;
String Elevation;
String ComputerRead;
String ZeroAz;

void setup() {
  // put your setup code here, to run once:
pinMode (Azpin, OUTPUT);
pinMode (Elpin, OUTPUT);
Serial.begin(9600);
AzServo.attach(Azpin);
ElServo.attach(Elpin);
ZeroAz = "";
}

void loop() {
  // put your main code here, to run repeatedly:

Azimuth = "";
Elevation = "";
ComputerRead = "";

  while(Serial.available()==0) {
    
  }
    ComputerRead= Serial.readString();  // read the incoming data as string
//    Serial.println(ComputerRead);     // echo the reception for testing purposes
  
// looking for command <AZxxx.x>
    for (int i = 0; i <= ComputerRead.length(); i++) {
     if ((ComputerRead.charAt(i) == 'A') && (ComputerRead.charAt(i+1) == 'Z')){ // if read AZ
      for (int j = i+2; j <= ComputerRead.length(); j++) {
        if (isDigit(ComputerRead.charAt(j))) {                                // if the character is number
          Azimuth = Azimuth + ComputerRead.charAt(j);
        }
        else {break;}
      }
     }
    }

// looking for command <ELxxx.x>
    for (int i = 0; i <= (ComputerRead.length()-2); i++) {
      if ((ComputerRead.charAt(i) == 'E')&&(ComputerRead.charAt(i+1) == 'L')){ // if read EL
        if ((ComputerRead.charAt(i+2)) == '-') {
                  break;
        }
        for (int j = i+2; j <= ComputerRead.length(); j++) {
          if (isDigit(ComputerRead.charAt(j))) {                               // if the character is number
            Elevation = Elevation + ComputerRead.charAt(j);
          }
          else {break;}
        }
      }
    }

// looking for command <ZOxxx.x>
    for (int i = 0; i <= (ComputerRead.length()-2); i++) {
      if ((ComputerRead.charAt(i) == 'Z')&&(ComputerRead.charAt(i+1) == 'O')){ // if read ZO
        if ((ComputerRead.charAt(i+2)) == '-') {
                  break;
        }
        for (int j = i+2; j <= ComputerRead.length(); j++) {
          if (isDigit(ComputerRead.charAt(j))) {                               // if the character is number
            ZeroAz = ZeroAz + ComputerRead.charAt(j);
          }
          else {break;}
        }
      }
    }

// compute the pulse width to send to the azimuth servo    
Az_input = (Azimuth.toInt() + ZeroAz.toInt())%360;
if (Az_input<200) {
  Az=1506. - 4.4822 * Az_input;
} 
else {
  Az=1506. - 4.4822 * (Az_input - 360.);
}

// compute the pulse width to send to the elevation servo
// new formula for “factory fresh” Stingray-9’s
El_input = Elevation.toInt();
El = 1498.-10.1 * El_input;

// issue commands to the servos
AzServo.writeMicroseconds(Az);
delay(100);
ElServo.writeMicroseconds(El);
delay(100);
Serial.write("TURNDONE");
    
}
