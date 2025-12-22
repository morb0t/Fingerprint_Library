#include <Arduino.h>
#include "Fingerprint.h"

HardwareSerial mySerial(2); // Use UART2
Fingerprint finger(&mySerial, 0x00000000);

void setup(){
    Serial.begin(115200);
    mySerial.begin(57600, SERIAL_8N1, 16, 17); // RX=16, TX=17
    finger.begin(57600);

    if (finger.verifyPassword()) {
        Serial.println("Fingerprint sensor found and password verified!");
    } else {
        Serial.println("Did not find fingerprint sensor :(");
        while (1) { delay(1); }
    }

    Serial.println("Sensor initialized.");

    // change led colors
    finger.LEDCtrl(0x01, 0x02, 0x03, 5); // Example parameters
    // reset color to default green
    finger.LEDCtrl(true);

}

void loop(){
    char option;
    while (!Serial.available()) {
        delay(1);
    }
    option = Serial.read();
    Serial.read(); // consume newline
    if(option == '1'){
        Serial.println("Place your finger on the sensor...");
        while(finger.getImage() != FINGERPRINT_OK);
        Serial.println("Fingerprint image captured!");
        finger.downloadImage();
    }else if(option == '2'){
        Serial.println("Place your finger on the sensor...");
        while(finger.getImage() != FINGERPRINT_OK);
        if(finger.image2Tz() != FINGERPRINT_OK){
            Serial.println("Error converting image to template.");
            return;
        }

        Serial.println("Model generated from image. Extracting Features...");

        uint8_t tempalteBuffer[512];
        if(finger.getModel(1, tempalteBuffer) == FINGERPRINT_OK){
            Serial.println("Template extracted successfully!");
            Serial.println("==== TEMPLATE START ====");
            for (int i = 0; i < 512; i++) {
                if (tempalteBuffer[i] < 0x10) {
                    Serial.print("0");
                }
                Serial.print(tempalteBuffer[i], HEX);
            }
            Serial.println("\n==== TEMPLATE END ====");
        } else {
            Serial.println("Error extracting template.");
        }

    }
}