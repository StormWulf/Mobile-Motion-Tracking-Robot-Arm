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

void setup() {
    Xbee.begin(9600);
    reset();
}

void loop() {
    
}

// Reset arm to initial position
void reset() {
    Xbee.println(27, 'i');                  // Cancel any previous commands
    Xbee.println("#0 P1890 #1 P1500 #2 P1630 #3 P1300 #4 P1500 S50");
}
