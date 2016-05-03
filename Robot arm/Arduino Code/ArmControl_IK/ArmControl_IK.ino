#include <SoftwareSerial.h>
#include <Math.h>

/*
Servo# - Function
0 - X movement
1 - Z movement
2 - Y movement
3 - Wrist
4 - Gripper
31 - Front left wheel
30 - Back left wheel
16 - Front right wheel
17 - Back right wheel
*/

SoftwareSerial Xbee(2,3);   // Software serial for Xbee communication

int ledPin_3 = 13;

//Setup message bytes
byte inputByte_0;
byte inputByte_1;
byte inputByte_2;
byte inputByte_3;
byte inputByte_4;

// Servo max/min positions
const int max_x = 2130;
const int min_x = 1550;
const int max_y = 2000;
const int min_y = 800;
const int max_z = 1700;
const int min_z = 1000;

// Coordinate max/min positions
const float max_z_coord = 1.2;
const float min_z_coord = 0.85;
const float max_y_coord = 0.3;
const float min_y_coord = -0.2;

// Coordinated-Servo translation equation variables
const int x_m = 1160;
const int x_b = 1666;
const int y_m = -1740;
const int y_b = 1612;
const int z_m = 1428;
const int z_b = -214;

// IK variables
const float baseHeight = 5.5;
const float humerus = 5.75;
const float ulna = 7.375;
const float hand = 3.375;
float handDeg = 0.0;
float y_Pos = 0.0;
float z_Pos = 0.0;
float wrist_Pos = 0.0;

boolean connected = false;  // True when Kinect connected

//Setup
void setup() {
    pinMode(ledPin_3, OUTPUT);
    Serial.begin(9600);
    Xbee.begin(9600);
    digitalWrite(ledPin_3, HIGH);
    delay(250);
    digitalWrite(ledPin_3, LOW);
    delay(250);
    digitalWrite(ledPin_3, HIGH);
    delay(250);
    digitalWrite(ledPin_3, LOW);
    delay(250);
    reset();                // Set arm to initial position
}

// Scale Kinect coordinates to arm coordinates
void scaleCoord(float z, float y) {
    float z_Scaled, y_Scaled = 0;
    float zK = z;
    float yK = y;
//    if (zK >= max_z_coord-0.01) zK = max_z_coord-0.01;
//    else if (zK <= min_z_coord+0.1) zK = min_z_coord+0.1;
//    if (yK >= max_y_coord-0.01) yK = max_y_coord-0.01;
//    else if (yK <= min_y_coord+0.01) yK = min_y_coord+0.01;
//    z_Scaled = (-31.429*zK) + 41.714;
    z_Scaled = (-31.429*zK) + 42.714;
//    y_Scaled = (30*yK) + 7.5;
    y_Scaled = (34*yK) + 6.8;
//    if (y_Scaled >= 17.0) y_Scaled = 17.0;
//    if (y_Scaled <= 0.0) y_Scaled = 0.0;
//    if (z_Scaled >= 16.0) z_Scaled = 16.0;
//    if (z_Scaled <= 5.0) z_Scaled = 5.0;
    y_Pos = y_Scaled;
    z_Pos = z_Scaled;
    Serial.print("y scale: "); Serial.print(y_Pos); Serial.print("   z scale: "); Serial.println(z_Pos);
}

// Calculate servo angles (degrees) from arm coordinates using inverse kinematics
void IK(float z, float y) {
    float A1, A2, sw, z_Scaled, y_Scaled = 0;
    float offsetY = sin(radians(handDeg))*hand;
    float offsetZ = cos(radians(handDeg))*hand;
    float wristY= y - offsetY - baseHeight;
    float wristZ = z - offsetZ;
    sw = sqrt((wristY*wristY)+(wristZ*wristZ));
    if (sw > (ulna+humerus)) sw = ulna+humerus;
    A1 = atan2( wristY, wristZ );
//    A1 = atan2( wristZ, wristY );
    A2 = acos(((humerus*humerus)-(ulna*ulna)+(sw*sw))/((2*humerus)*sw));
    z_Scaled = degrees(A1+A2);
    y_Scaled = -(180-degrees(acos(((humerus*humerus)+(ulna*ulna)-(sw*sw))/((2*humerus)*ulna))));
//    if (y_Scaled < 0) y_Scaled = y_Scaled + 180;
    z_Pos = z_Scaled;
    y_Pos = y_Scaled;
    wrist_Pos = handDeg - y_Pos - z_Pos;
    Serial.print("y angle: "); Serial.print(y_Pos); Serial.print("   z angle: "); Serial.println(z_Pos);
}

// Calculate servo positions from servo angles
void anglesToPos(float z, float y) {
    float y_Scaled, z_Scaled, wrist_Scaled = 0;
    z_Scaled = (7.0968*z) + 964.6;
    y_Scaled = (-10.56*y) + 750;
    wrist_Scaled = (9.4444*wrist_Pos) + 1350;
    z_Pos = z_Scaled;
    y_Pos = y_Scaled;
    wrist_Pos = wrist_Scaled;
    Serial.print("y pos: "); Serial.print(y_Pos); Serial.print("   z pos: "); Serial.println(z_Pos);
}

//Main Loop
void loop() {
    // Connect to Kinect
    if (!connected){
        connect();
    }
    // When connected, wait for serial input
    else{
        if (Serial.available() > 0){
            String serialIn = Serial.readStringUntil('\n');
            
            // Reset signals
            if (serialIn == "reset"){   // Reset signal
                reset();                // Reset arm to initial position
                connected = false;
                return;
            }
            if (serialIn == "PosReset") {
                reset();
            }
            
            // Gripper signals
            if (serialIn == "HandClosed") {
                closeGripper();
            }
            else if (serialIn == "HandOpened") {
                openGripper();
            }
            
            // Platform signals
            if (serialIn == "forward") {
                forward();
            }
            else if (serialIn == "backward") {
                backward();
            }
            else if (serialIn == "left") {
                left();
            }
            else if (serialIn == "right") {
                right();
            }
            else if (serialIn == "stop") {
                stop();   
            }
            else {
                stop();   
            }
            
            if (serialIn == "Up") {
                if (wrist_Pos < 2500){
                    handDeg += 5.0;
                    float temp = handDeg + 83.37 - 62.773;
                    wrist_Pos = (temp * 9.444) + 1350;
                    moveArm(1840, 1630, 1410);
                }
                
            }
            else if (serialIn == "Down") {
                if (wrist_Pos > 500) {
                    handDeg -= 5.0;
                    float temp = handDeg + 83.37 - 62.773;
                    wrist_Pos = (temp * 9.444) + 1350;
                    moveArm(1840, 1630, 1410);  
                }
                
            }
            
            // Parse positions from serial input
            int posX = serialIn.indexOf(',', 1);
            int posY = serialIn.indexOf(',', posX + 1);
            int posZ = serialIn.indexOf(',', posY + 1);
            String joint = serialIn.substring(0, posX);
            String x = serialIn.substring(posX+1, posY);
            String y = serialIn.substring(posY+1, posZ);
            String z = serialIn.substring(posZ+1);
            
            if (joint == "HandRight"){                   // Right hand instruction
                scaleCoord(z.toFloat(), y.toFloat());    // Scale kinect y/z coords to arm coords
                IK(z_Pos, y_Pos);                        // Calculate servo angles from arm coords
                anglesToPos(z_Pos, y_Pos);               // Calculate servo positions from servo angles
                float x_Pos = linReg(x.toFloat());
                moveArm(x_Pos, y_Pos, z_Pos);
//                moveX(x.toFloat());                      // Move X servo to position
//                moveY(y_Pos);                            // Move Y servo to position
//                moveZ(z_Pos);                            // Move Z servo to position
          }
          else if(joint == "Override") {
                moveArm(x.toFloat(), y.toFloat(), z.toFloat());
          }
        }
        // No serial input
        else {
            stop();      // Make sure platform stops moving
        }
    }
}

float linReg( float x ) {
    float moveTo = (x_m*x) + x_b;               // Translate kinect coordinates to servo positions
    if (moveTo > max_x) moveTo = max_x;         // Clamp values higher than max
    else if (moveTo < min_x) moveTo = min_x;    // Clamp values lower than min
    return moveTo;
} 

void moveArm( float x, float y, float z ) {
    Xbee.println(27, 'i');                      // Cancel any previous commands
    Xbee.print("#3 P");
    Xbee.print(wrist_Pos);
    Xbee.print(" S600 ");
    
//    float moveTo = (x_m*x) + x_b;               // Translate kinect coordinates to servo positions
//    if (moveTo > max_x) moveTo = max_x;         // Clamp values higher than max
//    else if (moveTo < min_x) moveTo = min_x;    // Clamp values lower than min
    Xbee.print("#0 P");
    Xbee.print(x);
    Xbee.print(" S600 ");
    
//    if (y > max_y) y = max_y;                   // Clamp values higher than max
//    else if (y < min_y) y = min_y;              // Clamp values lower than min
    Xbee.print("#2 P");
    Xbee.print(y);
    Xbee.print(" S600 ");
    
//    if (z > max_z) z = max_z;                   // Clamp values higher than max
//    else if (z < min_z) z = min_z;              // Clamp values lower than min
    Xbee.print("#1 P");
    Xbee.print(z);
    Xbee.print(" S600");
    Xbee.println(" T1");
}

// Move X servo to specified position
//void moveX(float x){
//    Xbee.println(27, 'i');                      // Cancel any previous commands
//    float moveTo = (x_m*x) + x_b;               // Translate kinect coordinates to servo positions
//    if (moveTo > max_x) moveTo = max_x;         // Clamp values higher than max
//    else if (moveTo < min_x) moveTo = min_x;    // Clamp values lower than min
//    Xbee.print("#0 P");
//    Xbee.print(moveTo);
//    Xbee.print(" S600");
////    Xbee.println("");
//}
//
//void moveY(float y) {
//    Xbee.println(27, 'i');                      // Cancel any previous commands
//    if (y < max_y) y = max_y;                   // Clamp values higher than max
//    else if (y > min_y) y = min_y;              // Clamp values lower than min
//    Xbee.print(" #2 P");
//    Xbee.print(y);
//    Xbee.print(" S600");
////    Xbee.println("");
//}
//
//void moveZ(float z) {
//    Xbee.println(27, 'i');                      // Cancel any previous commands
//    if (z > max_z) z = max_z;                   // Clamp values higher than max
//    else if (z < min_z) z = min_z;              // Clamp values lower than min
//    Xbee.print(" #1 P");
//    Xbee.print(z);
//    Xbee.print(" S600");
//    Xbee.println(" T10");
//}

// Connect to Kinect
void connect(){
    //Read Buffer
    if (Serial.available() == 5) 
    {
        //Read buffer
        inputByte_0 = Serial.read();
        delay(100);    
        inputByte_1 = Serial.read();
        delay(100);      
        inputByte_2 = Serial.read();
        delay(100);      
        inputByte_3 = Serial.read();
        delay(100);
        inputByte_4 = Serial.read();   
    }
    //Check for start of Message
    if(inputByte_0 == 16)
    {       
       //Detect Command type
       switch (inputByte_1) 
       {
          case 127:
             //Set PIN and value
             switch (inputByte_2)
            {
              case 4:
                if(inputByte_3 == 255)
                {
                  digitalWrite(ledPin_3, HIGH); 
                  break;
                }
                else
                {
                  digitalWrite(ledPin_3, LOW); 
                  break;
                }
              break;
            } 
            break;
          case 128:
            //Say hello
            Serial.print("HELLO FROM ARDUINO");
            break;
        } 
        //Clear Message bytes
        inputByte_0 = 0;
        inputByte_1 = 0;
        inputByte_2 = 0;
        inputByte_3 = 0;
        inputByte_4 = 0;
        //Let the PC know we are ready for more data
        Serial.print("-READY TO RECEIVE");
        digitalWrite(ledPin_3, HIGH);
        delay(250);
        digitalWrite(ledPin_3, LOW);
        delay(250);
        digitalWrite(ledPin_3, HIGH);
        delay(250);
        digitalWrite(ledPin_3, LOW);
        delay(250);
        digitalWrite(ledPin_3, HIGH);
        delay(250);
        digitalWrite(ledPin_3, LOW);
        connected = true;
    }
    else {

    }
}

// Reset arm and platform to initial position
void reset() {
    float temp = handDeg + 83.37 - 62.773;
    wrist_Pos = (temp * 9.444) + 1350;
    Xbee.print("#3 P"); Xbee.print(wrist_Pos);
    Xbee.println(" #0 P1840 #1 P1410 #2 P1630 #4 P2500 T500");
    Xbee.println("#31 P0 #30 P0 #16 P0 #17 P0");
}

// Open gripper
void openGripper() {
    Xbee.println("#4 P1500 S700 T1");
}

// Close gripper
void closeGripper() {
    Xbee.println("#4 P2500 S700 T1");
}

// Move platform forward
void forward(){
    Xbee.println("#31 P1600 #30 P1600 #16 P1300 #17 P1300");
}

// Move platform backward
void backward(){
    Xbee.println("#31 P1390 #30 P1390 #16 P1580 #17 P1580");
}

// Turn platform left. Left wheels backward, right wheels forward
void left(){ 
    Xbee.println("#31 P1300 #30 P1300 #16 P1300 #17 P1300");
}
// Turn platform right. Right wheels backward, left wheels forward
void right(){
    Xbee.println("#31 P1600 #30 P1600 #16 P1600 #17 P1600");
}

// Stop platform
void stop() {
    Xbee.println("#31 P0 #30 P0 #16 P0 #17 P0");
}
