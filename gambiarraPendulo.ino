#define kppen 30
#define kipen 1
#define kdpen 1.1
#define kpcar 5.6
#define kicar 0
#define kdcar 1.8
#define setPointAngle 180

int PWM = 6;
int dir1 = 5;
int dir2 = 4;
int channelApen = 18;
int channelBpen = 19;
int channelAcar = 20;
int channelBcar = 21;
int setPointCar = 0;
volatile long counterpen = 0L;
volatile long countercar = 0L;
double angle = 0;
double distance = 0;
double previoustime = 0.00;
double interval = 0.00;
double previousErrorAngle = 0.0, previousErrorCar = 0.0;
int valPot = 0;
double controlSignal = 0.0;
double pidAngle = 0.00;
double pidCar = 0.0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(channelApen, INPUT_PULLUP);
  pinMode(channelBpen, INPUT_PULLUP);
  pinMode(channelAcar, INPUT_PULLUP);
  pinMode(channelBcar, INPUT_PULLUP);

  pinMode(PWM, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(channelAcar), ai2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channelBcar), ai3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channelApen), ai0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channelBpen), ai1, CHANGE);
  counterpen = 0L;
  while (abs(counterpen) != 1200) {}
  counterpen = abs(counterpen);


}
void loop() {
  if (counterpen > 2400) {
    counterpen = counterpen - 2400;
  }
  else if (counterpen < -2400) {
    counterpen = counterpen + 2400;
  }
  valPot =  analogRead(A0);
  setPointCar = map(valPot, 0, 1023, -40, 40);
  angle = 0.15 * counterpen;
  distance = ((3.14 * 1.5) * countercar / (2400));
  interval = (micros() - previoustime) / 1000000.00;
  previoustime = micros();
  pidAngle = CalculaPID(angle, setPointAngle, kppen, kipen, kdpen, &previousErrorAngle);
  pidCar =  CalculaPID(distance, setPointCar, kpcar + 1.5, kicar, kdcar, &previousErrorCar);
  controlSignal = pidAngle - pidCar;
  if (controlSignal > 0) {
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
  }
  else {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
  }
  delay(15);
  analogWrite(PWM, constrain(abs(controlSignal) + 75, 0, 255));
  while (abs(angle) < 90 || abs(angle) > 270 )analogWrite(PWM, 0);
  

}
double CalculaPID(double feedback, double setPoint, double kp, double ki, double kd, double *previouserror) { //powered by Estacio
  double proportional = 0.0;       //Proportional, integral and derivative control signals initially set to zero.
  double integral = 0.0;
  double derivative = 0.0;
  double error = 0.0;

  error = setPoint - feedback;
  proportional = kp * error;
  integral = constrain(ki * (integral + error * interval), -255, 255);
  derivative = kd * (error - *previouserror) / interval;
  *previouserror = error;
  return (proportional + integral + derivative);
}
void ai0() {
  if (digitalRead(channelApen) != digitalRead(channelBpen)) {
    counterpen++;
  }
  else {
    counterpen--;
  }
}
void ai1() {
  if (digitalRead(channelApen) == digitalRead(channelBpen)) {
    counterpen++;
  }
  else {
    counterpen--;
  }
}
void ai2() {
  if (digitalRead(channelAcar) != digitalRead(channelBcar)) {
    countercar++;
  }
  else {
    countercar--;
  }
}
void ai3() {
  if (digitalRead(channelAcar) == digitalRead(channelBcar)) {
    countercar++;
  }
  else {
    countercar--;
  }
}
