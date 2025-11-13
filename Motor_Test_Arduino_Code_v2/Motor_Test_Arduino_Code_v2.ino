int motor2pin1 = 10;
int motor2pin2 = 9;

int motor1pin1 = 5;
int motor1pin2 = 6;

int lSpeed = 0;
int rSpeed = 0;

void setup() {
  Serial.begin(9600); // send and receive at 9600 baud
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  while (!Serial) ;
  Serial.println("enter speed between -100 and 100, on the form: 50,50  ");
}

void loop() {
  if (Serial.available() > 0) {
    String inputString = Serial.readString();
    inputString.trim();
    int commaindex = inputString.indexOf(",");
    lSpeed =(inputString.substring(0,commaindex)).toInt();
    rSpeed =  (inputString.substring(commaindex+1) ).toInt();
    }  
    runMotors(lSpeed, rSpeed);     
}

void runMotors(int leftSpeed,int rightSpeed){ //Function to run the motors, the Speeds can be set between -100 and 100
  if(rightSpeed>=0 and rightSpeed != 0){
    int Speed = map(rightSpeed, 0, 100, 0, 255);
    analogWrite(motor1pin1, Speed);
    digitalWrite(motor1pin2, LOW);    
  }
  if(rightSpeed<=0 and rightSpeed != 0){
    int Speed = map(rightSpeed, -100, 0, 255, 0);
    digitalWrite(motor1pin1, LOW);
    analogWrite(motor1pin2, Speed);
  }
  if(rightSpeed == 0){
   digitalWrite(motor1pin1, LOW);
   digitalWrite(motor1pin2, LOW);
  }
  if(leftSpeed>=0 and leftSpeed != 0){
    int Speed = map(leftSpeed, 0, 100, 0, 255);
    analogWrite(motor2pin1, Speed);
    digitalWrite(motor2pin2, LOW); 
  }
  if(leftSpeed<=0 and leftSpeed != 0){
    int Speed = map(leftSpeed, -100, 0, 255, 0);
    digitalWrite(motor2pin1, LOW);
    analogWrite(motor2pin2, Speed);
  }
  if(leftSpeed == 0){
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, LOW);   
  }  
}
