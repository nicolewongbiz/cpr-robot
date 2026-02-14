
void setup() {
  Serial.begin(9600);
}

void loop() {
  int hrValue = analogRead(A0);
  int brValue = analogRead(A1); 

  int hr = map(hrValue, 0, 1023, 60, 100); 
  int br = map(brValue, 0, 1023, 12, 20);

  Serial.print(hr);
  Serial.print(",");
  Serial.println(br);

  delay(100);
}