/*
#1 - reacher
#2 - forearm
#3 - wrist
#4 - gripper
*/

void setup() {
   Serial.begin(9600);
}

void loop() { 
   // Initial position
   Serial.println("#0 P970 #1 P1840 #2 P1640 #3 P1300 #4 P1500 S200");
   Serial.println("QP 0");
   //Serial.read();
   //while(1) { }
}
