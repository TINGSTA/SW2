void setup() {
  // put your setup code here, to run once:
  pinMode(7, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  
  digitalWrite(7, LOW);
  delay(1000);
  
  int c = 0;
  while (c < 5) {
    digitalWrite(7, HIGH);
    delay(100);
    digitalWrite(7, LOW);
    delay(100);
    c++;
  }
  digitalWrite(7, HIGH);

  while (1){
  }
  
}
