#define trigger1 5 // Trigger Pin
#define echo1    6 // Echo Pin
#define out      13
int inRange = 45; //Wide Range First sight of Target
int TargetRange = 12; //Minimal Parking Range to Target
const int NoiseReject = 25; //Percentage of reading closeness for rejection filter
long duration, distance, lastDuration, unfiltered, Sonar, RawSonar;
const unsigned int maxDuration = 11650; // around 200 cm, the sensor gets flaky at greater distances.
const long speed_of_sound = 29.1;    // speed of sound microseconds per centimeter

void setup() {
  pinMode(trigger1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(out,OUTPUT);
}

void loop() {
  SingleSonar();
  //Serial.println(Sonar);
  if(Sonar < 20){
    digitalWrite(out,HIGH);
  }
  else{
    digitalWrite(out,LOW);
  }

}

void SonarSensor(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  unfiltered = (duration / 2) / speed_of_sound; //Stores preliminary reading to compare
  if(duration <= 8) duration = ((inRange + 1) * speed_of_sound * 2); 
  //Rejects very low readings, kicks readout to outside detection range
  if(lastDuration == 0) lastDuration = duration;
  //Compensation parameters for intial start-up
  if(duration > (5 * maxDuration)) duration = lastDuration;
  //Rejects any reading defined to be out of sensor capacity (>1000)
  //Sets the fault reading to the last known "successful" reading
  if(duration > maxDuration) duration = maxDuration;  
  //Caps Reading output at defined maximum distance (~200)
  if((duration - lastDuration) < ((-1) * (NoiseReject / 100) * lastDuration)){
    distance = (lastDuration / 2) / speed_of_sound; //Noise filter for low range drops
  }
  distance = (duration / 2) / speed_of_sound;
  lastDuration = duration; //Stores "successful" reading for filter compensation
}
void SingleSonar() {
  SonarSensor(trigger1, echo1);
  Sonar = distance;
  RawSonar = unfiltered;
  delay(50); //Delay 50ms before next reading.
}
