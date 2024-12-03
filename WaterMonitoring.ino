#include <WiFi.h>
#include <ThingSpeak.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define sensor_pin 34 
const int analogInputPin = 35; // Example ADC pin
#define SensorPin 33      // pH meter Analog output to Arduino Analog Input 0

const char *ssid = "EMBEDDEDLAB";
const char *password = "1234567890";

const unsigned long YOUR_CHANNEL_ID = 2535462; // Change data type
const char *thingSpeakApiKey = "DYLM3BJRLY18MAMF";

float calibrationFactor = 0.210; // Example calibration factor
#define SensorPin 34      // pH meter Analog output to Arduino Analog Input 0
float calibration_value = 70.34 + 1.7;

unsigned long int avgValue;  // Store the average value of the sensor feedback
int buf[10], temp;

WiFiClient client; // Define a WiFiClient object

// Initialize the LCD with the I2C address 0x27 and 20x4 characters
LiquidCrystal_I2C lcd(0x27, 20, 4);

void setup() {
    Serial.begin(9600);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected to WiFi");
    ThingSpeak.begin(client); // Pass the client object to begin method
    
    pinMode(SensorPin, INPUT);
    pinMode(sensor_pin, INPUT);
  
    lcd.begin(20, 4); 
    lcd.init();
    lcd.backlight();
    Serial.begin(9600);
    lcd.clear();  
    lcd.setCursor(0, 0);  
    lcd.print("Welcome To");     
    lcd.setCursor(0, 1);  
    lcd.print("       pH Meter"); 
    delay(2000);
    lcd.clear();
}

void loop() {
    for(int i=0;i<10;i++) { // Get 10 sample value from the sensor for smoothing
        buf[i] = analogRead(SensorPin);
        delay(10);
    }
    for(int i=0;i<9;i++) { // Sort the analog from small to large
        for(int j=i+1;j<10;j++) {
            if(buf[i] > buf[j]) {
                temp=buf[i];
                buf[i]=buf[j];
                buf[j]=temp;
            }
        }
    }
    avgValue = 0;
    for(int i=2;i<8;i++) avgValue += buf[i];  // Take the average value of 6 center samples
    
    float phValue = (float)avgValue * 5.0 / 1024 / 6; // Convert the analog into millivolt
    phValue = -5.70 * phValue + calibration_value; // Convert the millivolt into pH value
    
    lcd.setCursor(0, 0);  
    lcd.print("pH Value: ");
    lcd.print(phValue); 
    Serial.println(phValue);
    
    int sensorValue = analogRead(analogInputPin);
    float tdsValue = sensorValue * calibrationFactor;
    
    // Print TDS value to LCD
    lcd.setCursor(0, 1);
    lcd.print("TDS: ");
    lcd.print(tdsValue);

    int value = analogRead(32);
    Serial.println(value);
    lcd.setCursor(0, 2);
    lcd.print("W Level :");
    lcd.print(value);
  
    if (value == 0) {
        lcd.print("  Empty ");
    } else if (value > 1 && value < 350) {
        lcd.print("  LOW   ");
    } else if (value > 350 && value < 510) {
        lcd.print("  Medium");
    } else if (value > 510){
        lcd.print("  HIGH  ");
    }

    int read_ADC = analogRead(sensor_pin);
    if(read_ADC > 208) read_ADC = 208;
    int ntu = map(read_ADC, 0, 208, 300, 0); 

    lcd.setCursor(0, 3);
    lcd.print("T: ");
    lcd.print(ntu);
    Serial.println(ntu);
    lcd.setCursor(6, 3); // Set cursor (column by row) indexing from 0
    if(ntu < 10)            lcd.print("Clean");
    if(ntu >= 10 && ntu < 30) lcd.print("Norm Clean");
    if(ntu >= 30)           lcd.print("Very Dirty");
    
    delay(200);
    
    // Update ThingSpeak fields with error handling
    int status1 = ThingSpeak.writeField(YOUR_CHANNEL_ID, 1, phValue, thingSpeakApiKey);
    int status2 = ThingSpeak.writeField(YOUR_CHANNEL_ID, 2, tdsValue, thingSpeakApiKey);
    int status3 = ThingSpeak.writeField(YOUR_CHANNEL_ID, 3, value, thingSpeakApiKey);
    int status4 = ThingSpeak.writeField(YOUR_CHANNEL_ID, 4, ntu, thingSpeakApiKey);
    
    // Check if data was successfully sent
    if (status1 != 200 || status2 != 200 || status3 != 200 || status4 != 200) {
        Serial.println("Error updating ThingSpeak fields");
    }
    
    delay(15000); // Delay before sending next data (ThingSpeak allows updates every 15 seconds)
}
