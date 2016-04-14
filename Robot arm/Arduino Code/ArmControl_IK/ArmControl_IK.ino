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
const int max_y = 800;
const int min_y = 2000;
const int max_z = 1700;
const int min_z = 1000;

// Coordinate max/min positions
const float max_z_coord = 1.2;
const float min_z_coord = 0.85;
const float max_y_coord = 0.25;
const float min_y_coord = -0.25;

// Coordinated-Servo translation equation variables
const int x_m = 967;
const int x_b = 1743;
const int y_m = -2175;
const int y_b = 1742;
const int z_m = 1667;
const int z_b = -333;

// IK variables
const float baseHeight = 5.5;
const float humerus = 5.75;
const float ulna = 7.375;
const float hand = 3.375;
const float handDeg = -15.0;
float y_Pos = 0;
float z_Pos = 0;

boolean connected = false;

//Setup
void setup() {
    pinMode(ledPin_3, OUTPUT);
    Serial.begin(9600);
    Xbee.begin(9600);
    digitalWrite(ledPin_3, HIGH);//
    delay(250);//
    digitalWrite(ledPin_3, LOW);//
    delay(250);//
    digitalWrite(ledPin_3, HIGH);//
    delay(250);//
    digitalWrite(ledPin_3, LOW);//
    delay(250);//
    
    // Set arm to initial position
    reset();
}

// Scale Kinect coordinate to arm coordinate
void scaleCoord(float z, float y) {
//    Serial.print("Z, Y: ");
//    Serial.print(z);
//    Serial.print(", ");
//    Serial.println(y);
    float z_Scaled = 0;
    float y_Scaled = 0;
    float zK = z;
    float yK = y;
    if (zK >= max_z_coord) zK = max_z_coord;
    else if (zK <= min_z_coord) zK = min_z_coord;
    if (yK >= max_y_coord-0.01) yK = max_y_coord-0.01;
    else if (yK <= min_y_coord+0.01) yK = min_y_coord+0.01;
    z_Scaled = (-31.429*zK) + 41.714;
    y_Scaled = (30*yK) + 7.5;
    if (y_Scaled >= 15.0) y_Scaled = 15.0;
//    Serial.print("Z, Y: ");
//    Serial.print(zK);
//    Serial.print(", ");
//    Serial.println(yK);
//    Serial.print("Scale: ");
//    Serial.print(z_Scaled);
//    Serial.print(", ");
//    Serial.println(y_Scaled);
    y_Pos = y_Scaled;
    z_Pos = z_Scaled;
}

// Calculate servo angles from scaled coordinates using inverse kinematics
void IK(float z, float y) {
    float A1, A2, sw = 0;
    float y_Scaled = 0;
    float z_Scaled = 0;
    float wrist1 = y -(sin(radians(handDeg))*hand) - baseHeight;
    float wrist2 = z -(cos(radians(handDeg))*hand);
//    Serial.print("Wrist1, Wrist2: ");
//    Serial.print(wrist1);
//    Serial.print(", ");
//    Serial.println(wrist2);
    sw = sqrt((wrist1*wrist1)+(wrist2*wrist2));
    if (sw > (ulna+humerus)) sw = ulna+humerus;
    A1 = atan2( wrist1, wrist2 );
    A2 = acos(((humerus*humerus)-(ulna*ulna)+(sw*sw))/((2*humerus)*sw));
//    Serial.print("A1, A2: ");
//    Serial.print(A1);
//    Serial.print(", ");
//    Serial.println(A2);
    z_Scaled = degrees(A1+A2);
    y_Scaled = -(180-degrees(acos(((humerus*humerus)+(ulna*ulna)-(sw*sw))/((2*humerus)*ulna))));
//    if (z_Scaled < 0) z_Scaled = z_Scaled + 270;
    if (y_Scaled < 0) y_Scaled = y_Scaled + 180;
//  Serial.print("IK: ");
//  Serial.print(z_Scaled);
//  Serial.print(", ");
//  Serial.println(y_Scaled);
  z_Pos = z_Scaled;
  y_Pos = y_Scaled;
}

// Calculate servo positions from servo angles
void anglesToPos(float z, float y) {
    float y_Scaled;
    float z_Scaled;
    z_Scaled = (9.333*z) + 860;
    y_Scaled = (-7.5*y) + 2000;
//  Serial.print("Pos: ");
//  Serial.print(z_Scaled);
//  Serial.print(", ");
//  Serial.println(y_Scaled);
    z_Pos = z_Scaled;
    y_Pos = y_Scaled;
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
            int posJoint = 0;
            
            if (serialIn == "reset"){   // Reset signal
                reset();                // Reset arm to initial position
                connected = false;
                return;
            }
            if (serialIn == "PosReset") {
                reset();
            }
            if (serialIn == "HandClosed") {
                closeGripper();
            }
            else if (serialIn == "HandOpened") {
                openGripper();
            }
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
            
            // Parse positions from serial input
            int posX = serialIn.indexOf(',', posJoint + 1);
            int posY = serialIn.indexOf(',', posX + 1);
            int posZ = serialIn.indexOf(',', posY + 1);
            String joint = serialIn.substring(posJoint, posX);
            String x = serialIn.substring(posX+1, posY);
            String y = serialIn.substring(posY+1, posZ);
            String z = serialIn.substring(posZ+1);
//            Serial.println("-----------------------");
//            Serial.println(serialIn);
//            Serial.print(z.toFloat());
//            Serial.print(", ");
//            Serial.println(y.toFloat());
            
            if (joint == "HandRight"){  // Right hand instruction
//                Serial.println("-------------------------");
//                Serial.print("Kinect coords: ");
//                Serial.print(x);
//                Serial.print(", ");
//                Serial.print(y);
//                Serial.print(", ");
//                Serial.println(z);
                scaleCoord(z.toFloat(), y.toFloat());
//                Serial.print("Arm coords: ");
//                Serial.print(y_Pos);
//                Serial.print(", ");
//                Serial.println(z_Pos);
                IK(z_Pos, y_Pos);
                anglesToPos(z_Pos, y_Pos);
//                Serial.print("Arm Pos: ");
//                Serial.print(y_Pos);
//                Serial.print(", ");
//                Serial.println(z_Pos);
                moveX(x.toFloat());     // Move X servo to position
                moveY_IK(y_Pos);
                moveZ_IK(z_Pos);
//                moveYZ(coords[1], coords[0]);
//                Xbee.println("#3 P1300 S100");   // Move wrist to safe position
            }
        }
        else {
            stop();
        }
    }
}

// Move X servo to specified position
void moveX(float x){
    Xbee.println(27, 'i');                  // Cancel any previous commands
    float moveTo = (x_m*x) + x_b;           // Translate kinect coordinates to servo positions
    if (moveTo > max_x) moveTo = max_x;     // Clamp values higher than max
    else if (moveTo < min_x) moveTo = min_x;    // Clamp values lower than min
    Xbee.print("#0 P");
    Xbee.print(moveTo);
    Xbee.print(" S500");
    Xbee.println("");
}

void moveY_IK(float y) {
    Xbee.println(27, 'i');                  // Cancel any previous commands
    if (y < max_y) y = max_y;     // Clamp values higher than max
    else if (y > min_y) y = min_y;    // Clamp values lower than min
    Xbee.print("#2 P");
    Xbee.print(y);
    Xbee.print(" S500");
    Xbee.println("");
}

void moveZ_IK(float z) {
    Xbee.println(27, 'i');                  // Cancel any previous commands
    if (z > max_z) z = max_z;     // Clamp values higher than max
    else if (z < min_z) z = min_z;    // Clamp values lower than min
    Xbee.print("#1 P");
    Xbee.print(z);
    Xbee.print(" S500");
    Xbee.println("");
}

void moveYZ(float y, float z) {
    Xbee.println(27, 'i');                  // Cancel any previous commands
//    if (y > max_y) y = max_y;
//    else if (y < min_y) y = min_y;
//    if (z > max_z) z = max_z;
//    elst if (z < min_z) z = min_z;
    Xbee.print("#2 P");
    Xbee.print(y);
    Xbee.print(" #1 P");
    Xbee.print(z);
    Xbee.print(" S500");
    Xbee.println("");
}

// Move Y servo to specified position
void moveY(float y){
    Xbee.println(27, 'i');                  // Cancel any previous commands
    float moveTo = (y_m*y) + y_b;           // Translate kinect coordinates to servo positions
    if (moveTo > max_y) moveTo = max_y;     // Clamp values higher than max
    else if (moveTo < min_y) moveTo = min_y;    // Clamp values lower than min
    Xbee.print("#2 P");
    Xbee.print(moveTo);
    Xbee.print(" S500");
    Xbee.println("");
}

// Move Z servo to specified position
void moveZ(float z){
    Xbee.println(27, 'i');                  // Cancel any previous commands
    float moveTo = (z_m * z) + z_b;         // Translate kinect coordinates to servo positions
    if (moveTo > max_z) moveTo = max_z;     // Clamp values higher than max
    else if (moveTo < min_z) moveTo = min_z;    // Clamp values lower than min
    Xbee.print("#1 P");
    Xbee.print(moveTo);
    Xbee.print(" S500");
    Xbee.println("");
}

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
}

// Reset arm to initial position
void reset() {
    Xbee.println("#0 P1840 #1 P1410 #2 P1630 #3 P1300 #4 P1500 T500");
    Xbee.println("#31 P0 #30 P0 #16 P0 #17 P0");
}

// Open gripper
void openGripper() {
    Xbee.println("#4 P1500 S700 T1");
}

// Close gripper
void closeGripper() {
    Xbee.println("#4 P2280 S700 T1");
}

void forward(){
	Xbee.println("#31 P1550 #30 P1550 #16 P1460 #17 P1460");
}

void backward(){
	Xbee.println("#31 P1460 #30 P1460 #16 P1550 #17 P1550");
}

//left wheels backward, right wheels forward
void left(){ 
	Xbee.println("#31 P1460 #30 P1460 #16 P1460 #17 P1460");
}
//right wheels backward, left wheels forward
void right(){
	Xbee.println("#31 P1550 #30 P1550 #16 P1550 #17 P1550");
}

void stop() {
    Xbee.println("#31 P0 #30 P0 #16 P0 #17 P0");
}
