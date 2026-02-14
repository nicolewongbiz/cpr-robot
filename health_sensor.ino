// Assuming Heart Rate on A0 and Breathing (Potentiometer/Flex) on A1
void setup() {
  Serial.begin(9600);
}

void loop() {
  int hrValue = analogRead(A0); // Replace with your sensor logic
  int brValue = analogRead(A1); // Replace with your sensor logic

  // Map raw values to usable numbers if necessary
  int hr = map(hrValue, 0, 1023, 60, 100); 
  int br = map(brValue, 0, 1023, 12, 20);

  // Send to Serial as "HR,BR"
  Serial.print(hr);
  Serial.print(",");
  Serial.println(br);

  delay(100); // 10Hz update rate is plenty for real-time
}