//Setup Output

int ledPin_3 = 13;

//Setup message bytes

byte inputByte_0;

byte inputByte_1;

byte inputByte_2;

byte inputByte_3;

byte inputByte_4;

boolean connected = false;

boolean cont = true;

int pos = 1640;

//Setup

void setup() {
  pinMode(ledPin_3, OUTPUT);
  Serial.begin(9600);
  digitalWrite(ledPin_3, HIGH);//
  delay(250);//
  digitalWrite(ledPin_3, LOW);//
  delay(250);//
  digitalWrite(ledPin_3, HIGH);//
  delay(250);//
  digitalWrite(ledPin_3, LOW);//
  delay(250);//
  Serial.println("#0 P920 #1 P1410 #2 P1630 #3 P1300 #4 P1500 S50");
}

//Main Loop

void loop() {
    if (!connected){
        connect();
//        if(pos < 2000){
//            pos = pos + 1;
//            Serial.print("#2 P");
//            Serial.print(pos);
//            Serial.print(" S50");
//            Serial.println("");
//        }
        // Initial position
        //Serial.println("#0 P970 #1 P1840 #2 P1640 #3 P1300 #4 P1500 S200");
    }
    else{
        if (Serial.available() > 0){
//            digitalWrite(ledPin_3, HIGH);//
//            delay(250);//
//            digitalWrite(ledPin_3, LOW);//
            
//            if(pos > 1200){
//                pos = pos - 1;
//                Serial.print("#2 P");
//                Serial.print(pos);
//                Serial.print(" S50");
//                Serial.println("");
//            }
            
//            String serialIn = Serial.readString();
            String serialIn = Serial.readStringUntil('\n');
//            if(serialIn == "Closing"){
//                connected = false;
//                digitalWrite(ledPin_3, HIGH);//
//                delay(250);//
//                digitalWrite(ledPin_3, LOW);//
//                delay(250);//
//                digitalWrite(ledPin_3, HIGH);//
//                delay(250);//
//                digitalWrite(ledPin_3, LOW);//
//                return;
//            }
            int posJoint = 0;
            int posX = serialIn.indexOf(',', posJoint + 1);
            int posY = serialIn.indexOf(',', posX + 1);
            int posZ = serialIn.indexOf(',', posY + 1);
            String joint = serialIn.substring(posJoint, posX);
            String x = serialIn.substring(posX+1, posY);
            String y = serialIn.substring(posY+1, posZ);
            String z = serialIn.substring(posZ+1);
            String data = joint + "," + x + "," + y + "," + z;
//            Serial.println(data);
            if (joint == "HandRight"){
                Serial.println("Arduino received command for right hand.");
                //if (y.toFloat() >=0){
//                    Serial.println(27, 'i');
                    moveY(y.toFloat());
                    moveX(x.toFloat());
                    moveZ(z.toFloat());
                   /* if () {
                      
                    } */
                //}
                //else{
//                    Serial.println(27, 'i');
                   // moveY(y.toFloat());   
               // }
//                Serial.println("#0 P700 #2 P1200 S200 T1000");
                
//                cont = false;
            }
            else{
                //Serial.println("New data received: " + serialIn);
            }
        }
    }
}

void moveY(float y){
    Serial.println(27, 'i');
    float moveTo = (-1483.333*y)+1525;
    if (moveTo > 1970) moveTo = 1970;
    else if (moveTo < 1080) moveTo = 1080;
    Serial.print("#2 P");
    Serial.print(moveTo);
    Serial.print(" S500 T100");
    Serial.println("");
}

void moveX(float x){
    Serial.println(27, 'i');
    float moveTo = (1183*x)+678;
    if (moveTo > 1270) moveTo = 1270;
    else if (moveTo < 560) moveTo = 560;
    Serial.print("#0 P");
    Serial.print(moveTo);
    Serial.print(" S500 T100");
    Serial.println("");
}

void moveZ(float z){
    Serial.println(27, 'i');
    float moveTo = (3133*z)-1647;
    if (moveTo > 1800) moveTo = 1800;
    else if (moveTo < 860) moveTo = 860;
    Serial.print("#1 P");
    Serial.print(moveTo);
    Serial.print(" S500 T100");
    Serial.println("");
}

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
    /*else{
        Serial.print("Data received" + inputByte_0);
    }*/
}
