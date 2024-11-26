int recievePin = 11;
int transmitPin = 12;
int message = 0;
int baudDelay = 200;
void setup() {
  // put your setup code here, to run once:
  pinMode(recievePin, INPUT);
  pinMode(transmitPin, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  message = 0;
  Serial.println("waiting for start bit");
  while(digitalRead(recievePin)){
  }

  Serial.println("recieved start bit");
  delay(baudDelay+50); // let start bit end, delay a bit
  for (int i=0; i<8; i++){
    message |= digitalRead(recievePin) << 7-i;
    delay(baudDelay);
  }

  Serial.println(message, BIN);

}
