void setup() {
  // put your setup code here, to run once:
  pinMode(21, OUTPUT);
  digitalWrite(21, HIGH);
  Serial2.begin(9600);
  while(!Serial2);
  digitalWrite(21, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(21, HIGH);
  delay(1000);
  digitalWrite(21, LOW);
  delay(1000);
  
}
