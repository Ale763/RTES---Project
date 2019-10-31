String ledStatus = "off";
int blinkRate = 0;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
}

void loop() {  
  Serial.println("Enter LED status (on/off):");
  while(Serial.available() == 0)
  {
    if(ledStatus == "on")
    {
      digitalWrite(LED_BUILTIN, HIGH);  
      delay(blinkRate);                       
      digitalWrite(LED_BUILTIN, LOW);   
      delay(blinkRate);
    }
    else if(ledStatus == "off")
    {
        digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    }
  }
  while(Serial.available() == 0){}
//  if (Serial.available() > 0) 
//    {
    // read the incoming byte:
    String input = Serial.readString();
    input.trim();
    if(input == "on")
    {
      ledStatus = input;
      Serial.println("Enter blink rate(1-60 sec):");
      while(Serial.available() == 0){}
      if (Serial.available() > 0) 
      {
        String blinkRateInput = Serial.readString();
        blinkRateInput.trim();
        Serial.println("You have selected LED on"+blinkRateInput+" sec");
        blinkRate = blinkRateInput.toInt()*1000;
      }
    } else if (input == "off") 
    {
      ledStatus = "off";
    }
    
//  }
}
