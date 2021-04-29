
//motor directory
#define CW  0
#define CCW 1

//define button
#define pushCal 12
#define pushErr 13
 
//motor control pin
#define motorDirPin 7
#define motorPWMPin 9
#define enablePin 8

//motor control 2 pin
#define motorDirPin2 5
#define motorPWMPin2 6
#define enablePin2 3
 
//encoder pin
#define encoderPinA 2
#define encoderPinB 4

//encoder pin
#define encoderPinA2 10
#define encoderPinB2 11

//deklarasi sensor
int sensorMax[6] = {550, 531, 598, 603, 521, 600};
int sensorMin[6] = {35, 33, 38, 40, 33, 36};
int sensor1 = A0;
int sensor2 = A1;
int sensor3 = A2;
int sensor4 = A3;
int sensor5 = A4;
int sensor6 = A5;
int bacaSensor[6];
int peka[6];
int sumPeka = 0;
int sumPeka2 = 0;
 
//encoder var
int encoderPos = 0;
int encoderPos2 = 0;

float Kp          = 3;
float Ki          = 0;
float Kd          = 6;
int   targetPos = 150;
int targetPos2 = 150;
int   error;
int   control;
int   velocity;
float   integral;
float   derivative;
float   dt = 0.01;
int   prevError;

int   error2;
int   control2;
int   velocity2; 

//external interrupt encoder
void doEncoderA()
{
  digitalRead(encoderPinB)?encoderPos--:encoderPos++;
}

//external interrupt encoder
void doEncoderA2()
{
  digitalRead(encoderPinB2)?encoderPos2--:encoderPos2++;
}

void kalibrasi(){

  bacaSensor[0] = analogRead(sensor1);
  bacaSensor[1] = analogRead(sensor2);
  bacaSensor[2] = analogRead(sensor3);
  bacaSensor[3] = analogRead(sensor4);
  bacaSensor[4] = analogRead(sensor5);
  bacaSensor[5] = analogRead(sensor6);

  for (int x = 0; x <= 5; x++){
      if(bacaSensor[x] > sensorMax[x]){
          //untuk mengkalibrasi nilai max
          sensorMax[x] = bacaSensor[x];
      }
      if(bacaSensor[x] < sensorMin[x]){
          //untuk mengkalibrasi nilai max
          sensorMin[x] = bacaSensor[x];
      }
    
    //Variabel untuk menentukan median
 	peka[x] = (sensorMax[x] + sensorMin[x])/2;
    sumPeka = sumPeka + peka[x];
    
    //cek nilai error
  }
  
  sumPeka2 = sumPeka/6; 
  
  for(int i=0; i<6; i++){
    Serial.print("Sensor");
    Serial.print(i+1);
    Serial.print(": ");
    Serial.print(bacaSensor[i]);
    Serial.print("\n");
  } 

}

void ErrorCheckUP(int peka){
  bacaSensor[0] = analogRead(sensor1);
  bacaSensor[1] = analogRead(sensor2);
  bacaSensor[2] = analogRead(sensor3);
  bacaSensor[3] = analogRead(sensor4);
  bacaSensor[4] = analogRead(sensor5);
  bacaSensor[5] = analogRead(sensor6);
  
   //Sensor 1 mendeteksi gelap sisanya terang
  if(bacaSensor[0] < peka && bacaSensor[1] > peka &&
     bacaSensor[2] > peka && bacaSensor[3] > peka &&
     bacaSensor[4] > peka && bacaSensor[5] > peka){
  
    error = -4;
  }
  
  //Sensor 1 dan 2 mendeteksi gelap sisanya terang
  if(bacaSensor[0] < peka && bacaSensor[1] < peka && 
     bacaSensor[2] > peka && bacaSensor[3] > peka &&
     bacaSensor[4] > peka && bacaSensor[5] > peka){
    
    error = -3;
    
  }
  
  //Sensor 2 mendeteksi gelap sisanya terang
  if(bacaSensor[0] > peka && bacaSensor[1] < peka && 
     bacaSensor[2] > peka && bacaSensor[3] > peka &&
     bacaSensor[4] > peka && bacaSensor[5] > peka){
    
    error = -2;
  }
  
   //Sensor 2 dan 3 mendeteksi gelap sisanya terang
  if(bacaSensor[0] > peka && bacaSensor[1] < peka && 
     bacaSensor[2] < peka && bacaSensor[3] > peka &&
     bacaSensor[4] > peka && bacaSensor[5] > peka){
    
    error = -1;
  }
  
  //Sensor 3 mendeteksi gelap sisanya terang
  if(bacaSensor[0] > peka && bacaSensor[1] > peka && 
     bacaSensor[2] < peka && bacaSensor[3] > peka &&
     bacaSensor[4] > peka && bacaSensor[5] > peka){
    
    error = 0;
  }
  
  //Sensor 3 dan 4 mendeteksi gelap sisanya terang
  if(bacaSensor[0] > peka && bacaSensor[1] > peka && 
     bacaSensor[2] < peka && bacaSensor[3] < peka &&
     bacaSensor[4] > peka && bacaSensor[5] > peka){
    
    error = 0;
  }
  
  //Sensor 4 mendeteksi gelap sisanya terang
  if(bacaSensor[0] > peka && bacaSensor[1] > peka && 
     bacaSensor[2] > peka && bacaSensor[3] < peka &&
     bacaSensor[4] > peka && bacaSensor[5] > peka){
    
    error = 0;
  }
  
  //Sensor 4 dan 5 mendeteksi gelap sisanya terang
  if(bacaSensor[0] > peka && bacaSensor[1] > peka && 
     bacaSensor[2] > peka && bacaSensor[3] < peka &&
     bacaSensor[4] < peka && bacaSensor[5] > peka){
    
    error = 1;
  }
  
  //Sensor 5 mendeteksi gelap sisanya terang
  if(bacaSensor[0] > peka && bacaSensor[1] > peka && 
     bacaSensor[2] > peka && bacaSensor[3] > peka &&
     bacaSensor[4] < peka && bacaSensor[5] > peka){
    
    error = 2;
  }
  
  //Sensor 5 dan 6 mendeteksi gelap sisanya terang
  if(bacaSensor[0] > peka && bacaSensor[1] > peka && 
     bacaSensor[2] > peka && bacaSensor[3] > peka &&
     bacaSensor[4] < peka && bacaSensor[5] < peka){
    
    error = 3;
  }
  if(bacaSensor[0] > peka && bacaSensor[1] > peka && 
     bacaSensor[2] > peka && bacaSensor[3] > peka &&
     bacaSensor[4] > peka && bacaSensor[5] < peka){

    error = 4;
  }
  
   Serial.print("Error: ");
   Serial.println(error);
}
 
void setup()
{
  //setup interrupt
    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);
    pinMode(encoderPinA2, INPUT_PULLUP);
    pinMode(encoderPinB2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA,RISING);
    attachInterrupt(digitalPinToInterrupt(encoderPinA2), doEncoderA2,RISING);
  
    //setup motor driver
    pinMode(motorDirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, HIGH);

    //setup motor driver
    pinMode(motorDirPin2, OUTPUT);
    pinMode(enablePin2, OUTPUT);
    digitalWrite(enablePin2, HIGH);
  	
  	//setup push button
  	pinMode(pushCal, INPUT);
  	pinMode(pushErr, INPUT);
  

    Serial.begin(9600);
}

 
void loop()
{	
  if(digitalRead(pushCal) == LOW){
    kalibrasi();
  }
  
  if(digitalRead(pushErr) == LOW){
    ErrorCheckUP(sumPeka2);
  }
    //potentiometer sebagai penentu targetpos
    error   = targetPos - encoderPos; 
    integral += error * dt;
    derivative = (error - prevError)/dt;
    control = (Kp*error) + (Ki*integral) + (Kd*derivative);

    //potentiometer sebagai penentu targetpos
    error2   = targetPos2 - encoderPos2; 
    integral += error * dt;
    derivative = (error - prevError)/dt;
    control2 = (Kp*error) + (Ki*integral) + (Kd*derivative);

    velocity = min(max(control, -255), 255);
    velocity2 = min(max(control2, -255), 255); 

    if(velocity >= 0)
    {
        digitalWrite(motorDirPin, CW); //output 
        analogWrite(motorPWMPin, velocity); //output duty 
    }
    else
    {
        digitalWrite(motorDirPin, CCW);
        analogWrite(motorPWMPin, 255+velocity);
    }

    if(velocity2 >= 0)
    {
        digitalWrite(motorDirPin2, CW); //output 
        analogWrite(motorPWMPin2, velocity2); //output duty 
    }
    else
    {
        digitalWrite(motorDirPin2, CCW);
        analogWrite(motorPWMPin2, 255+velocity2);
    }
  
    
   // Serial.println(encoderPos);
}