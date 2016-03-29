//Setup Output

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


#include <SoftwareSerial.h>
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
              reset();                  // Reset arm to initial position     
              return;
            }
            
            // Parse positions from serial input
            int posX = serialIn.indexOf(',', posJoint + 1);
            int posY = serialIn.indexOf(',', posX + 1);
            int posZ = serialIn.indexOf(',', posY + 1);
            String joint = serialIn.substring(posJoint, posX);
            String x = serialIn.substring(posX+1, posY);
            String y = serialIn.substring(posY+1, posZ);
            String z = serialIn.substring(posZ+1);
            String data = joint + "," + x + "," + y + "," + z;
            
            if (joint == "HandRight"){  // Right hand instruction
                Serial.println("Arduino received command for right hand.");
                moveY(y.toFloat());     // Move Y servo to position
                moveX(x.toFloat());     // Move X servo to position
                moveZ(z.toFloat());     // Move Z servo to position
                Xbee.println("#3 P1300");   // Move wrist to safe position
            }
        }
    }
}

// Move Y servo to specified position
void moveY(float y){
    Xbee.println(27, 'i');                  // Cancel any previous commands
//    float moveTo = ((-1483.333*y)+1525);  // Original equation
    float moveTo = (-1112*y)+1525;          // Slower movement
    if (moveTo > max_y) moveTo = max_y;     // Clamp values higher than max
    else if (moveTo < min_y) moveTo = min_y;    // Clamp values lower than min
    Xbee.print("#2 P");
    Xbee.print(moveTo);
    Xbee.print(" S600 T10");
    Xbee.println("");
}

// Move X servo to specified position
void moveX(float x){
    Xbee.println(27, 'i');                  // Cancel any previous commands
//    float moveTo = (2167*x)+1417;         // Original equation
    float moveTo = (1625*x)+1525;           // Slower movement
    if (moveTo > max_x) moveTo = max_x;     // Clamp values higher than max
    else if (moveTo < min_x) moveTo = min_x;    // Clamp values lower than min
    Xbee.print("#0 P");
    Xbee.print(moveTo);
    Xbee.print(" S600 T10");
    Xbee.println("");
}

// Move Z servo to specified position
void moveZ(float z){
    Xbee.println(27, 'i');                  // Cancel any previous commands
//    float moveTo = ((3133*z)-1647);       // Original equation
    float moveTo = (1880*z)-456;            // Slower movement
    if (moveTo > max_z) moveTo = max_z;     // Clamp values higher than max
    else if (moveTo < min_z) moveTo = min_z;    // Clamp values lower than min
    Xbee.print("#1 P");
    Xbee.print(moveTo);
    Xbee.print(" S600 T10");
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
void reset(){
  Xbee.println("#0 P1890 #1 P1410 #2 P1630 #3 P1300 #4 P1500 S50");
  connected = false;
}
