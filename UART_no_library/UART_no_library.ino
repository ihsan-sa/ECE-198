int recievePin = 11;
int transmitPin = 12;
int message = 0;

int16_t sendMsg = 6942;

int baudDelay = 100;
int bufferDelay = 500;
void setup() {
  // put your setup code here, to run once:
  pinMode(recievePin, INPUT);  
  digitalWrite(transmitPin, 1);
  pinMode(transmitPin, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:

//recieve
//  message = 0;
//  Serial.println("waiting for start bit");
//  while(digitalRead(recievePin)){
//  }
//
//  Serial.println("recieved start bit");
//  delay(baudDelay+10); // let start bit end, delay a bit
//  for (int i=0; i<16; i++){
//    message |= digitalRead(recievePin) << 15-i;
//    delay(baudDelay);
//  }
//
//  Serial.println(message, BIN);

// send
  digitalWrite(transmitPin, 1);
  delay(bufferDelay);
  digitalWrite(transmitPin, 0);
  delay(baudDelay);

  // send message
  for (int i=0; i<16; i++){
    digitalWrite(transmitPin, !!(sendMsg & (1 << 15-i)) );
    Serial.println(!!(sendMsg & (1 << 15-i)));
    delay(baudDelay);
  }
  Serial.println("");
  // go back to default high
  
}
