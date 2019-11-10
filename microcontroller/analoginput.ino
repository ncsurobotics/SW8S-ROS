int analogPin = A10;     
int digitalPin = P4_3;
int valAnalog = 0;       // variable to store the read value
int valDigital = 0; 

void setup()
{
  Serial.begin(9600);              //  setup serial
  pinMode(digitalPin, INPUT);      // sets the digital pin 7 as input
  pinMode(analogPin, INPUT); 
   pinMode(RED_LED, OUTPUT);

}

void loop()
{
  valAnalog = analogRead(analogPin);     // read the input pin
  Serial.print("AnalogValue:");
  Serial.println(valAnalog);
  valDigital = digitalRead(digitalPin);     // read the input pin
  Serial.print("DigitalValue:");
  Serial.println(valDigital);
  delay(1000);
}
