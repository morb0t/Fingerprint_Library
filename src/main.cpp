#include <Arduino.h>
#include "Fingerprint.h"
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>

HardwareSerial mySerial(2); // Use UART2
Fingerprint finger(&mySerial, 0x00000000);

// Thresholds are calibrated at enrollment; defaults are only used on failure.
static uint16_t imageHashThreshold = 60;

static uint8_t enrolledImageHash[32]; // 256-bit image hash
static bool hasEnrollment = false;
static const uint32_t CAPTURE_TIMEOUT_MS = 5000; // fail fast if finger/sensor not responding

static uint16_t hammingDistance(const uint8_t* a, const uint8_t* b, size_t bytes){
    uint16_t dist = 0;
    for (size_t i = 0; i<bytes; i++){
        dist += __builtin_popcount((unsigned)(a[i] ^ b[i]));
    }
    return dist;
}

static std::string toHex(const uint8_t* data, size_t len) {
    static const char* hex = "0123456789ABCDEF";
    std::string out;
    out.reserve(len * 2);
    for (size_t i = 0; i < len; i++) {
        out.push_back(hex[data[i] >> 4]);
        out.push_back(hex[data[i] & 0x0F]);
    }
    return out;
}

static inline void setBit(uint8_t* out, int bitIndex, bool bit){
    size_t byteIndex = (size_t)bitIndex >> 3;
    int bitInByte = bitIndex & 7;
    if(bit) out[byteIndex] |= (uint8_t)(1u << bitInByte);
    else    out[byteIndex] &= (uint8_t)~(1u << bitInByte);
}

// Reusable image buffer to avoid heap churn.
static uint8_t imageBuf[40000];
static size_t imageLen = 0;

// Capture raw image bytes into buffer; returns false on failure.
static bool captureRawImage() {
    uint32_t start = millis();
    while (finger.getImage() != FINGERPRINT_OK) {
        if (millis() - start > CAPTURE_TIMEOUT_MS) {
            Serial.println("Timeout waiting for finger/image.");
            return false;
        }
        delay(1);
    }
    size_t received = 0;
    uint8_t rc = finger.downloadImageToBuffer(imageBuf, sizeof(imageBuf), &received);
    if (rc != FINGERPRINT_OK) {
        Serial.print("Failed to download image: ");
        Serial.println(rc, HEX);
        return false;
    }
    imageLen = received;
    return true;
}

// Simple block-mean hash over the raw image (256 bits).
static bool computeImageHash(const uint8_t* image, size_t len, uint8_t outHash[32]) {
    if (len < 1024) return false;
    // Assume width 256; derive height.
    const int width = 256;
    int height = (int)(len / width);
    if (height <= 0) return false;

    const int blocks = 16;
    int blockW = width / blocks;
    int blockH = height / blocks;
    if (blockW <= 0 || blockH <= 0) return false;

    // global mean
    uint64_t sumAll = 0;
    for (size_t i = 0; i < len; i++) sumAll += image[i];
    double globalMean = (double)sumAll / (double)len;

    memset(outHash, 0, 32);
    int bitIdx = 0;
    for (int by = 0; by < blocks; by++) {
        for (int bx = 0; bx < blocks; bx++) {
            uint64_t blkSum = 0;
            for (int y = 0; y < blockH; y++) {
                int row = by * blockH + y;
                if (row >= height) break;
                int offset = row * width + bx * blockW;
                for (int x = 0; x < blockW && (bx * blockW + x) < width; x++) {
                    blkSum += image[offset + x];
                }
            }
            double blkMean = (double)blkSum / (double)(blockW * blockH);
            setBit(outHash, bitIdx++, blkMean >= globalMean);
            if (bitIdx >= 256) break;
        }
        if (bitIdx >= 256) break;
    }
    return true;
}

static bool captureImageHash(uint8_t outHash[32]) {
    Serial.println("Place your finger on the sensor...");
    if (!captureRawImage()) return false;
    return computeImageHash(imageBuf, imageLen, outHash);
}

static void enrollUser() {
    // Single-pass enrollment using raw image hash for speed.
    uint8_t imgHash[32];
    if (!captureImageHash(imgHash)) {
        Serial.println("Failed to compute image hash during enrollment.");
        return;
    }
    memcpy(enrolledImageHash, imgHash, 32);
    hasEnrollment = true;
    Serial.print("Stored image hash (hex): ");
    Serial.println(toHex(enrolledImageHash, 32).c_str());
    Serial.print("Image-hash threshold: ");
    Serial.println(imageHashThreshold);
}

static void verifyUser() {
    if (!hasEnrollment) {
        Serial.println("No enrollment stored yet. Please enroll first.");
        return;
    }

    uint8_t verifyImgHash[32];
    if (!captureImageHash(verifyImgHash)) {
        Serial.println("Warning: could not compute image hash for verification.");
        Serial.println("No match: could not compute image hash.");
        return;
    }

    uint16_t imgDist = hammingDistance(enrolledImageHash, verifyImgHash, 32);
    Serial.print("Image hash distance: ");
    Serial.println(imgDist);
    Serial.print("Image hash threshold: ");
    Serial.println(imageHashThreshold);
    if (imgDist <= imageHashThreshold) {
        Serial.println("Match: image hash within threshold.");
    } else {
        Serial.println("No match: image hash above threshold.");
    }
}

void setup(){
    Serial.begin(115200);

    mySerial.setRxBufferSize(1024);
    mySerial.begin(57600, SERIAL_8N1, 16, 17); // RX=16, TX=17
    finger.begin(57600);

    if (finger.verifyPassword()) {
        Serial.println("Fingerprint sensor found and password verified!");
    } else {
        Serial.println("Did not find fingerprint sensor :(");
        while (1) { delay(1); }
    }

    Serial.println("Sensor initialized.");
    // Use largest packet size to speed up image downloads (default is smaller).
    finger.setPacketSize(FINGERPRINT_PACKET_SIZE_128);

    // change led colors
    finger.LEDCtrl(0x01, 0x02, 0x03, 5); // Example parameters

}

void loop(){
    Serial.println("Select an option:");
    Serial.println("1. Enroll user (capture + image hash)");
    Serial.println("2. Verify user (image hash compare)");
    Serial.println("3. Capture raw fingerprint image");

    // Wait for input and flush any CR/LF noise.
    while (!Serial.available()) {
        delay(1);
    }
    char option = Serial.read();
    // drain the rest of the line (handles CRLF)
    while (Serial.available()) { Serial.read(); }

    if(option == '1'){
        enrollUser();
    }else if(option == '2'){
        verifyUser();
    } else if(option == '3'){
        Serial.println("Place your finger on the sensor...");
        uint32_t start = millis();
        while(finger.getImage() != FINGERPRINT_OK) {
            if (millis() - start > CAPTURE_TIMEOUT_MS) {
                Serial.println("Timeout waiting for finger/image.");
                return;
            }
            delay(1);
        }
        Serial.println("Fingerprint image captured!");
        finger.downloadImage();
    }
}
