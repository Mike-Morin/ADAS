 #include <Wire.h>
bool state = false;
void setup() {
    Serial.begin(9600);

  // put your setup code here, to run once:
  Wire.begin(9);                // join i2c bus with address #9
  Wire.setClock(10000);
  Wire.onRequest(requestEvent); // register event


}

void loop() {
  // put your main code here, to run repeatedly:
//Serial.println("Boop.");
//delay(500);
   delay(100);
}

void requestEvent() {
  Wire.write(millis()); // respond with message of 2 bytes
  Serial.println("event");
  //state = !state;
  //digitalWrite(17, state);
  // as expected by master
}
