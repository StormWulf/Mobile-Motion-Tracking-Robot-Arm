//Setup Output

int ledPin_3 = 13;

//Setup message bytes

byte inputByte_0;

byte inputByte_1;

byte inputByte_2;

byte inputByte_3;

byte inputByte_4;

boolean connected = false;

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

   //Serial.println("QP #0");
}

//Main Loop

void loop() {
    if (!connected){
        connect();
        // Initial position
        //Serial.println("#0 P970 #1 P1840 #2 P1640 #3 P1300 #4 P1500 S200");
    }
    else{
        if (Serial.available()){
            String serialIn = Serial.readString();
            String sub1 = serialIn.substring(0, serialIn.indexOf('.', 0));
            if (sub1 == "HandRight"){
                Serial.println("Arduino received command for right hand.");
                Serial.println("#0 P1000 S200");
            }
            else{
                Serial.println("New data received: " + serialIn);
            }
        }
    }
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
