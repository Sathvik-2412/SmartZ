#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

const int ledPin = 13; // the pin that the LED is attached to
int incomingByte;// a variable to read incoming serial data into
int temp;
int door=0; //0->close, 1->open
int in1=2;
int in2=3;
const int pirPin = 8;
int set_timer=5;
int seconds =0;
long delay_Start;
int x=0;

int pirState = LOW;
int val = 0;

void setup() {
  //Initialize the LCD
  lcd.begin();
  lcd.clear();
  //Turn on the backlight
  lcd.backlight();
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  //initialize the PIR pin as input
  pinMode(pirPin, INPUT);
    // initialize serial communication:
  Serial.begin(115200);
  if (!mlx.begin()) {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  };
   pinMode(in1,OUTPUT); 
  pinMode(in2,OUTPUT); 
  //door open sequence
  digitalWrite(in1, HIGH);  
  digitalWrite(in2, LOW);
  delay(2000); 
  digitalWrite(in1, LOW); 
  digitalWrite(in2, LOW);
  door=1;
  //door stays open for 2 seconds
  delay(2000);
  //door close sequence
  digitalWrite(in1, LOW);  
  digitalWrite(in2, HIGH);
  delay(2000); 
  digitalWrite(in1, LOW); 
  digitalWrite(in2, LOW); 
  door=0;
  lcd.println("Welcome  ");
}

void loop() {
  

val = digitalRead(pirPin);
if (val == HIGH)
{
  digitalWrite(ledPin, HIGH);
  if (pirState == LOW)
  {Serial.write("SEND");
    
    pirState = HIGH;
       // see if there's incoming serial data:
  while(Serial.available() <= 0); 
    // read the oldest byte in the serial buffer:
    incomingByte = Serial.read();
    
    
    // if it's a capital H (ASCII 72), turn on the LED:
    if (incomingByte == 'H') {
      //digitalWrite(ledPin, HIGH);
      lcd.clear();
      lcd.println("Wear your mask ");
      delay(10);
      pirState = LOW;
      //Serial.write('D');
    }
    // if it's an L (ASCII 76) turn off the LED:
    else if (incomingByte == 'L') {
      //digitalWrite(ledPin, LOW);
      delay(10);
      lcd.clear();
      lcd.println("mask confirm ");
      temp = mlx.readObjectTempF();
      if (temp>96) {
        lcd.clear();
        lcd.print("Temp: ");
        lcd.print(temp);
        lcd.print("F  ");
       //delay(2000);
        if (temp > 98.6){
          lcd.setCursor(0,1);
          lcd.println("Fever,step aside");
          goto exit;
        }
      delay(2000);
      lcd.clear();
      lcd.println("Opening door ");
      //door open sequence if door is closed (door=0)
      if(door==0){
        digitalWrite(in1, HIGH); 
        digitalWrite(in2, LOW); 
        delay(2000); 
        digitalWrite(in1, LOW); 
        digitalWrite(in2, LOW);
        door=1;
      }

      
      
      
      //Serial.write('D');
    }
  
  }
  seconds=0;

  delay_Start = millis();
}

else 
{
  //wait for 2 seconds after last detected motion
      if((pirState==HIGH) && door==1 && (millis()- delay_Start) > 999)
      { 
      seconds = seconds+1;  
      delay_Start = millis(); 
      }
      if((pirState==HIGH) && door==1 && seconds>set_timer)
      { 
        //door close sequence
      lcd.clear();
      lcd.println("Closing door");
      digitalWrite(ledPin, HIGH);
      digitalWrite(in1, LOW);  
      digitalWrite(in2, HIGH); 
      delay(2000); 
      digitalWrite(in1, LOW); 
      digitalWrite(in2, LOW);
      door=0; 
      
      pirState = LOW;
      seconds=0;

      }
  
}
exit:
}
