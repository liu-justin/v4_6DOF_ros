int minorStepsPerRev = 800; // written on the motor driver
int minorStepsPerMajorStep = minorStepsPerRev/200; // how many minor steps are in between the 200 major steps of a standard stepper motor

double radPerMajorStep = PI/100.0; // 2PI rads/200 steps
double radPerMinorStep = radPerMajorStep/float(minorStepsPerMajorStep);

int vel = 1;
unsigned long previousTime = millis();

void setup() {
  // put your setup code here, to run once:

  Serial.begin(57600);
  Serial.println(minorStepsPerMajorStep);
  Serial.println(radPerMajorStep,10);
  Serial.println(radPerMinorStep,10);
  Serial.println(radPerMinorStep/float(vel),10);
}

void loop() {
  unsigned long time = millis();
  // put your main code here, to run repeatedly:
  
  if ((time-previousTime)/1000.0 > radPerMinorStep/float(vel)){
//    Serial.print(time-previousTime);
//    Serial.println("hit!");
    previousTime = millis();
  }
}
