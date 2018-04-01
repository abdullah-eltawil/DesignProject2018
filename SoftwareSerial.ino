/*
  Software serial MSE 2202 IR tester

 The circuit:

* RX is digital pin 7 (connect to TX of other device)
* TX is digital pin 11 (connect to RX of other device)

*/


#include <SoftwareSerial.h>

SoftwareSerial mySerial(7, 11); // RX, TX

char pyramidLetter;
void setup() {

  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("MSE 2202 IR tester");

  // set the data rate for the SoftwareSerial port
  mySerial.begin(2400);
  //mySerial.println("Hello, world?");
}

 

void loop() { // run over and over

  if (mySerial.available())
  {
    //Serial.write(mySerial.read());
     
    pyramidLetter = mySerial.read();
    if(pyramidFound()){
      foundPyramid();
    }
    else
      searchForPyramid();
  }
  else
    searchForPyramid();
}//

void searchForPyramid(){
  Serial.println("Searching!");
}
bool pyramidFound(){
  if(pyramidLetter == 'A' || pyramidLetter == 'E')
    return true;
  else
    return false;
}
void foundPyramid(){
  Serial.println("Found Pyramid!");
}

