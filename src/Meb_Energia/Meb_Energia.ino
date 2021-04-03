int killPinIn = A2; // arbitrary
int killPinOut = 3; // arbitrary
bool killed = false;
int incomingByte = 0;

void setup()
{
  Serial.begin(115200);
  pinMode(killPinIn, INPUT);
}

void loop()
{   
  killed = digitalRead(killPinIn);
  while(Serial.available() > 0) {
      int incomingByte = Serial.parseInt();
      if (incomingByte == 34) {
          killed = true;
      }
      if (killed) {
          digitalWrite(killPinOut, HIGH);
          Serial.println('1');
          killed = false;
      }
  }
  
}
