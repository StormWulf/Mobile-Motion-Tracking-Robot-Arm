void setup() {
  // open the serial port:
  Serial.begin(9600);
  // initialize control over the keyboard:
//  Keyboard.begin();
}

void loop() {
  // check for incoming serial data:
//  if (Serial.available() > 0) {
//    // read incoming serial data:
//    char inChar = Serial.read();
//    // Type the next ASCII value from what you received:
//    Keyboard.write(inChar + 1);
//	while( inChar == 'W' ){
//	        forward();
//	}
//
//	while( inChar == 'A' ){
//		left();
//	}
//
//	while( inChar == 'S' ){
//		backward();
//	}
//
//	while( inChar == 'D' ){
//		right();
//	}
//  }
    
    if (Serial.available() > 0) {
        String serialIn = Serial.readStringUntil('\n');
        Serial.println("serialIn: " + serialIn);
    }
    
}

void forward(){
	Serial.println("#31 P1555 #30 P1555 #16 P1555 #17 P1555");
}

void backward(){
	Serial.println("#31 P1460 #30 P1460 #16 P1460 #17 P1460");
}

//left wheels backward, right wheels forward
void left(){ 
	Serial.println("#30 P1460 #31 P1460 #16 P1555 #17 P1555");
}
//right wheels backward, left wheels forward
void right(){
	Serial.println("#16 P1460 #17 P1460 #30 P1555 #31 P1555");
}
