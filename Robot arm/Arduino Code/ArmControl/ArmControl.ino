#include <SoftwareSerial.h>

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
int min_y = 1080;
int max_y = 1970;
int min_x = 1200;
int max_x = 2500;
int min_z = 860;
int max_z = 1800;

// Coordinated-Servo translation equation variables
int x_m = 1885;
int x_b = 1472;
int y_m = -1242;
int y_b = 1525;
int z_m = 2000;
int z_b = -700;

boolean connected = false;
int count = 0;

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
    
//    Xbee.println("#31 PO10 #30 PO-5");
    
    // Set arm to initial position
    reset();
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
            
            if (joint == "HandRight"){  // Right hand instruction
//                moveTo(x.toFloat(), y.toFloat(), z.toFloat());
                Xbee.println("#3 P1300 S100");   // Move wrist to safe position
                moveY(y.toFloat());     // Move Y servo to position
                moveX(x.toFloat());     // Move X servo to position
                moveZ(z.toFloat());     // Move Z servo to position
            }
        }
        else {
            stop();
        }
    }
}

// Move servo positions to updated Kinect coordinates
void moveTo(float x, float y, float z) {
    Xbee.println(27, 'i');                  // Cancel any previous commands
    float moveTo_X = (x_m * x) + x_b;
    float moveTo_Y = (y_m * y) + y_b;
    float moveTo_Z = (z_m * z) + z_b;
    if (moveTo_X > max_x) moveTo_X = max_x;     // Clamp values higher than max
    else if (moveTo_X < min_x) moveTo_X = min_x;    // Clamp values lower than min
    if (moveTo_Y > max_y) moveTo_Y = max_y;     // Clamp values higher than max
    else if (moveTo_Y < min_y) moveTo_Y = min_y;    // Clamp values lower than min
    if (moveTo_Z > max_z) moveTo_Z = max_z;     // Clamp values higher than max
    else if (moveTo_Z < min_z) moveTo_Z = min_z;    // Clamp values lower than min
    Xbee.print("#0 P");
    Xbee.print(moveTo_X);
    Xbee.print(" S800 ");
    Xbee.print("#2 P");
    Xbee.print(moveTo_Y);
    Xbee.print(" S800 ");
    Xbee.print("#1 P");
    Xbee.print(moveTo_Z);
    Xbee.print(" S800");
    Xbee.println(" T1");
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
    Xbee.println("#0 P1890 #1 P1410 #2 P1630 #3 P1300 #4 P1500 T500");
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
