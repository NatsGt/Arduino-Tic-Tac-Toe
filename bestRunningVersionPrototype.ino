



#include <AccelStepper.h>


enum Type {
    NONE = 0,
    CROSS, 
    CIRCLE, 
};

// Boards setup
int Board[3][3] =  {{NONE, NONE, NONE}, {NONE, NONE, NONE}, {NONE,NONE,NONE}};


int reservePieces[5][5] = { {NONE, CIRCLE, NONE, NONE, CROSS}, {CIRCLE, NONE, NONE, NONE, CROSS}, {CIRCLE,NONE,NONE, NONE, CROSS},{CIRCLE, NONE, NONE, NONE, CROSS}, {NONE, NONE, NONE, NONE, NONE}};

int winningX1 = -1;
int winningX2 = -1;
int winningX3 = -1;

int winningY1 = -1;
int winningY2 = -1;
int winningY3 = -1;

int winningXs[3] = {1, -1, -1};
int distanceBetweenSquares = 28; // distance in mm
int pitch = 7; //allegedly we do 2mm per revolute. we do 200steps per revolute 

bool winnerIsCircle = false;

const int motorXdirPin = 4;
const int motorXstepPin =5;
const int motorYdirPin = 2;
const int motorYstepPin =3;


const int solenoidPin = 11;
const int magnetPin = 10;

const int homePinX = 13; 
const int homePinY = 12;



AccelStepper stepperX(AccelStepper::DRIVER, motorXstepPin, motorXdirPin); // This is the motor connected to the LOWER parts. it's in the BLUE corner
AccelStepper stepperY(AccelStepper::DRIVER, motorYstepPin, motorYdirPin); // BLACK corner motor

bool Homing = true;
bool HomingX = true;
bool HomingY = true;
bool currentMoveDone = false;
long initial_homing=-1;
bool isCircleTurn = false;
bool isWinningState = false;
int movesMade = 0;
Type winner = NONE;

int sensorPin = A0;

bool isGoalState(){ // checks if there is a winner in software
    if ((Board[0][0] == Board[0][1] && Board[0][1] == Board[0][2]) && Board[0][0] != 0) {
        winner = Type(Board[0][0]);
        Serial.println("first");
        winningX1 = 1;
        winningX2 = 1;
        winningX3 = 1;
        winningY1 = 1;
        winningY2 = 2;
        winningY3 = 3;
        return true;
    } else if ((Board[1][0] == Board[1][1] && Board[1][1] == Board[1][2]) && Board[1][0] != 0)
    {
      Serial.println("second");
        winner = Type(Board[1][0]);
        winningX1 = 2;
        winningX2 = 1;
        winningX3 = 1;
        winningY1 = 1;
        winningY2 = 2;
        winningY3 = 3;
        return true;
    } else if ((Board[2][0] == Board[2][1] && Board[2][1] == Board[2][2]) && Board[2][0] != 0)
    {
      Serial.println("third");
        winner = Type(Board[2][0]);
        winningX1 = 3;
        winningX2 = 3;
        winningX3 = 3;
        winningY1 = 1;
        winningY2 = 2;
        winningY3 = 3;
        return true;
    } else if ((Board[0][0] == Board[1][0] && Board[1][0] == Board[2][0]) && Board[0][0] != 0)
    {
      Serial.println("fourth");
        winner = Type(Board[0][0]);
        winningX1 = 1;
        winningX2 = 2;
        winningX3 = 3;
        winningY1 = 1;
        winningY2 = 1;
        winningY3 = 1;
        return true;
    }else if ((Board[0][1] == Board[1][1] && Board[1][1] == Board[1][2]) && Board[0][1] != 0)
    {
      Serial.println("fifth");
        winner = Type(Board[0][1]);
        winningX1 = 1;
        winningX2 = 2;
        winningX3 = 2;
        winningY1 = 2;
        winningY2 = 2;
        winningY3 = 3;        
        return true;
    } else if ((Board[0][2] == Board[1][2] && Board[1][2] == Board[2][2]) && Board[0][2] != 0)
    {
      Serial.println("6th");
        winner = Type(Board[0][2]);
        winningX1 = 1;
        winningX2 = 2;
        winningX3 = 3;
        winningY2 = 3;
        winningY2 = 3;
        winningY3 = 3;
        return true;
        
    } else if ((Board[0][0] == Board[1][1] && Board[1][1] == Board[2][2]) && Board[0][0] != 0)
    {
      //diagonal /
      Serial.println("7th");
        winner = Type(Board[0][0]);
        winningX1 = 1;
        winningX2 = 2;
        winningX3 = 3;
        winningY1 = 1;
        winningY2 = 2;
        winningY3 = 3;        
        return true;
    } else if ((Board[2][0] == Board[1][1] && Board[1][1] == Board[0][2]) && Board[2][0] != 0)
    {
      // reverse diagonal
      Serial.println("8th");
        winner = Type(Board[2][0]);
        winningX1 = 3;
        winningX2 = 2;
        winningX3 = 1;
        winningY1 = 1;
        winningY2 = 2;
        winningY3 = 3;        
        return true;
    }
    return false;    
}

void extendSolenoid(){
    digitalWrite(solenoidPin, LOW);
  }

void retractSolenoid(){
    digitalWrite(solenoidPin, HIGH);  
  }

long getPosXFromSquare(int square){
  switch (square) {
    case 0:
        return 0;
    case 1:
        return 0-(1*(distanceBetweenSquares/pitch)*200);
    case 2:
    
        return 0-(2*(distanceBetweenSquares/pitch)*200);
    case 3:// This should be used to pick up unused pieces
        return 0-(3*(distanceBetweenSquares/pitch)*200);  
    case 4:// This should be used to pick up unused pieces
        return 0-(4*(distanceBetweenSquares/pitch)*200);                                                    
    }
  
  }
long getPosYFromSquare(int square){
  switch (square) {
    case 0:
        return 0;
    case 1:
        return 0+(1*(distanceBetweenSquares/pitch)*200);
    case 2:
        return 0+(2*(distanceBetweenSquares/pitch)*200);
    case 3:
        return 0+(3*(distanceBetweenSquares/pitch)*200); // This should be used to pick up unused pieces
    case 4: 
        return 0+(4*(distanceBetweenSquares/pitch)*200); // And this
                                                    
    }
  
  }


void homing() { // dont touch
  retractSolenoid();
  delay(100);
  // Move away from motor untill switch is presset
   while (digitalRead(homePinX)) {  // Make the Stepper move CCW until the switch is activated   
    stepperX.moveTo(-initial_homing);  // Set the position to move to (THIS SHOULD BE + TO MOVE IT AWAY FROM THE MOTOR)
    initial_homing--;  // Increase by 1 for next move if needed 
    stepperX.run();  // Start moving the stepper
    delay(1);
  // put your setup code here, to run once:
      }
   stepperX.setCurrentPosition(0);
   stepperX.setMaxSpeed(300.0);
   initial_homing=1;
  while (!digitalRead(homePinX)) { // Move the box away from the switch untill it is relased
    stepperX.moveTo(-initial_homing);  
    stepperX.run();
    initial_homing++;
    delay(5);
  }
    stepperX.setCurrentPosition(0);
    Serial.println("Homing X Completed");      
    
    
    // HOMINGY

    initial_homing=1;
    while (digitalRead(homePinY)) {
    stepperY.moveTo(-initial_homing);  // THIS SHOULD BE - TO MOVE IT AWAY FROM THE MOTOR)
    initial_homing++;  // Decrease by 1 for next move if needed 
    stepperY.run();  // Start moving the stepper
    delay(1);
  // put your setup code here, to run once:
      }
    initial_homing = 1;
    stepperY.setCurrentPosition(0);
    stepperY.setMaxSpeed(300.0);
  while (!digitalRead(homePinY)) { // Move the box away from the switch untill it is relased
    stepperY.moveTo(initial_homing);  
    stepperY.run();
    initial_homing++;
    delay(5);
  }
    stepperY.setCurrentPosition(0);      
  }


float getSensorReading(){
  int sensorValue = analogRead(sensorPin);  // Read the analog value from the sensor
  float voltage = sensorValue * (5.0 / 1023.0);  // Convert analog value to voltage
  float distance = 27.86 * pow(voltage, -1.15);
  return distance;
}

bool PickUpPiece(bool circle){
  Type toPickUp = NONE;
  if (circle){toPickUp = CIRCLE;
  
  }
  else{
    toPickUp = CROSS;
    }
  long squareX = -1;
  long squareY = -1;  
  
  for (long i = 0; i<6; i++){
    for (long j = 0; j < 5; j++){
        if (reservePieces[i][j] == toPickUp){
          squareX = i;
          squareY = j;
          reservePieces[i][j] = NONE;
          Serial.println(i);
          Serial.println(j);
          goto pieceFound;
          
          }
      }
    }
  pieceFound:
  if (squareX >= 0 && squareY >= 0){
 long targetPosX = getPosXFromSquare(squareX);
 long targetPosY = getPosYFromSquare(squareY); 
  stepperX.moveTo(targetPosX);
  stepperX.setSpeed(2000);

  stepperY.moveTo(targetPosY);
  stepperY.setSpeed(2000);


  while(stepperX.distanceToGo() != 0){
  stepperX.runSpeedToPosition();    
    }
  while(stepperY.distanceToGo() != 0){
    stepperY.runSpeedToPosition();
    }
  extendSolenoid();
  delay(100);
  digitalWrite(magnetPin, HIGH);    
  Serial.println("i should be done moving");
    }
  else {
    Serial.println("you dun goofed m8, no unused pieces left");
    
    }
  delay(300);
   return true;
}

bool moveSensorToSquare(int squareX, int squareY){
  long currentPosX = stepperX.currentPosition();
  long targetPosX = getPosXFromSquare(squareX);
  long targetPosY = getPosYFromSquare(squareY - 1);
  //Serial.println(targetPosX);
  
  currentMoveDone = false;
    
  stepperX.moveTo(targetPosX);
  stepperX.setSpeed(2000);  

  stepperY.moveTo(targetPosY);
  stepperY.setSpeed(2000);

  while(stepperX.distanceToGo() != 0){
  stepperX.runSpeedToPosition();    
    }
  while(stepperY.distanceToGo() != 0){
    stepperY.runSpeedToPosition();
    }
  return true;
  }

bool moveToSquare(int squareX, int squareY){
  long currentPosX = stepperX.currentPosition();
  long targetPosX = getPosXFromSquare(squareX);
  long targetPosY = getPosYFromSquare(squareY);
  //Serial.println(targetPosX);
  
  currentMoveDone = false;
    
  stepperX.moveTo(targetPosX);
  stepperX.setSpeed(2000);  

  stepperY.moveTo(targetPosY);
  stepperY.setSpeed(2000);

  while(stepperX.distanceToGo() != 0){
  stepperX.runSpeedToPosition();    
    }
  while(stepperY.distanceToGo() != 0){
    stepperY.runSpeedToPosition();
    }
  if (isCircleTurn){
    Board[squareX-1][squareY-1] = CIRCLE;
    }
  else {
    Board[squareX-1][squareY-1] = CROSS;
    }
  return true;
  }

  bool checkWinningSpace(int x, int y){
    
    bool isMoved = moveSensorToSquare(x,y);
        while(!isMoved){
      }
    delay(1000);
    float reading = getSensorReading();
    if(reading < 10){
    //if(reading < 10 && !winnerIsCircle){
      Serial.println("this is a cross");
      return true;
    } else if(reading >12 && reading <15){
    //} else if(reading >12 && reading <15 && winnerIsCircle){
      Serial.println("this is a circle");
      return true;
    }
    Serial.println("nothing here");
    return false;
  }


 bool checkWinningState(){
    retractSolenoid();
    digitalWrite(magnetPin, LOW); 
    bool res1 = checkWinningSpace(winningX1,winningY1);
    if(res1 == false){
    return false;
    }
    bool res2 = checkWinningSpace(winningX2,winningY2);
    if(!res2){
      return false;
    }
    bool res3 = checkWinningSpace(winningX3,winningY3);
    if(!res3){
      return false;
    }
    return true;
  }

void setup() {
  Serial.begin(9600);
  pinMode(homePinX, INPUT_PULLUP); // limit switch to GND
  pinMode(homePinY, INPUT_PULLUP); // limit switch to GND
  pinMode(magnetPin, OUTPUT);
  pinMode(solenoidPin, OUTPUT);
  stepperX.setMaxSpeed(10000);
  stepperY.setMaxSpeed(1000);
  stepperX.setAcceleration(60.0);
  stepperY.setAcceleration(60.0);  
  digitalWrite(magnetPin, LOW);
  
    // HOMING
   delay(5);  // Wait for driver to wake up
   //HOMING
   homing();      
   //reverseHoming();
//   stepperX.setCurrentPosition(0); // this is only needed when we don't do homing and it might not even be needed there
//   stepperY.setCurrentPosition(0); // this is only needed when we don't do homing and it might not even be needed there
}

void loop() {
  
  
  while(Serial.available()>0 && isWinningState == false){
    digitalWrite(magnetPin, LOW);
    String cmd = Serial.readStringUntil('!');
    int xPos = -1;
    int yPos = -1;
    if (cmd.length() == 0){
      xPos = -1;
      yPos = -1;     
      }
    else {
       xPos = cmd[0] - '0';
       yPos = cmd[2] - '0'; 
      
      }
    if (xPos >= 0 && yPos >= 0){
    retractSolenoid();
    delay(1000);
    isCircleTurn = !isCircleTurn;
    bool move_finished = false;      
    
    delay(2000);
    bool pickUpPiece = PickUpPiece(isCircleTurn); // This picks up an unused piece that it then wants to use

    while (!pickUpPiece){
      }
    }
    delay(500);
    retractSolenoid();
    
    bool isMoved = moveToSquare(xPos, yPos);
    while(!isMoved){
      
      }
    delay(5);
    extendSolenoid();
    delay(5);
    digitalWrite(magnetPin, LOW);
    delay(500);

    isWinningState = isGoalState();
    while(Serial.read()>=0);
      }
    //Serial.println(isWinningState);
      //isWinningState = isGoalState();
    if (isWinningState){
      if (isCircleTurn){
        winnerIsCircle = true;
      }
      bool winChecker = checkWinningState();
      if (winChecker){
        Serial.println("winner confirmed");
        }
     else {
       Serial.println("winner couldn't be confirmed.");
       
      }
      
    }

  // retractSolenoid();
  // float read = getSensorReading();
  // Serial.println(read);
  // delay(1000);

}




int mmToSteps(int mm){
  return 200*mm; 
  }


  
void reverseHoming(){ // Used for debugging + testing purposes
   while (digitalRead(homePinX)) {  // Make the Stepper move CCW until the switch is activated   
    stepperX.moveTo(-initial_homing);  // Set the position to move to (THIS SHOULD BE + TO MOVE IT AWAY FROM THE MOTOR)
    initial_homing++;  // Increase by 1 for next move if needed 
    stepperX.run();  // Start moving the stepper
    delay(1);
      }  

    // HOMINGY
    while (digitalRead(homePinY)) {
    stepperY.moveTo(initial_homing);  // THIS SHOULD BE - TO MOVE IT AWAY FROM THE MOTOR)
    initial_homing++;  // Decrease by 1 for next move if needed 
    stepperY.run();  // Start moving the stepper
    delay(1);
      }  
  
  
  }
// What we do in homing is we start by moving one box away from the motor untill the switch is pressed, then untill the button is released, the box moves away from the swithc
// Then we do the same for the 2nd motor and set the position of both of them to 0.
