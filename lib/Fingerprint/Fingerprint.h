#ifndef __FINGERPRINT_H__
#define __FINGERPRINT_H__

#include <Arduino.h>

#if defined(__AVR__) || defined(ESP8266)
    #include <SoftwareSerial.h>
#endif

// Fingerprint sensor command and response codes
#define FINGERPRINT_OK 0x00
#define FINGERPRINT_PACKETRECIEVEERR 0x01
#define FINGERPRINT_NOFINGER 0x02
#define FINGERPRINT_IMAGEFAIL 0x03
#define FINGERPRINT_IMAGEMESS 0x06
#define FINGERPRINT_FEATUREFAIL 0x07
#define FINGERPRINT_NOMATCH 0x08
#define FINGERPRINT_NOTFOUND 0x09
#define FINGERPRINT_ENROLLMISMATCH 0x0A
#define FINGERPRINT_BADLOCATION 0x0B
#define FINGERPRINT_DBRANGEFAIL 0x0C
#define FINGERPRINT_UPLOADFEATUREFAIL 0x0D
#define FINGERPRINT_PACKETRESPONSEFAIL 0x0E
#define FINGERPRINT_UPLOADFAIL 0x0F
#define FINGERPRINT_DELETEFAIL 0x10
#define FINGERPRINT_DBCLEARFAIL 0x11
#define FINGERPRINT_PASSFAIL 0x13
#define FINGERPRINT_INVALIDIMAGE 0x15
#define FINGERPRINT_FLASHERR 0x18
#define FINGERPRINT_INVALIDREG 0x1A
#define FINGERPRINT_ADDRCODE 0x20
#define FINGERPRINT_PASSVERIFY 0x21
#define FINGERPRINT_STARTCODE 0xEF01
#define FINGERPRINT_COMMANDPACKET 0x1
#define FINGERPRINT_DATAPACKET 0x2
#define FINGERPRINT_ACKPACKET 0x7
#define FINGERPRINT_ENDDATAPACKET 0x8
#define FINGERPRINT_TIMEOUT 0xFF
#define FINGERPRINT_BADPACKET 0xFE
#define FINGERPRINT_GETIMAGE 0x01
#define FINGERPRINT_IMAGE2TZ 0x02
#define FINGERPRINT_SEARCH 0x04
#define FINGERPRINT_REGMODEL 0x05
#define FINGERPRINT_STORE 0x06
#define FINGERPRINT_LOAD 0x07
#define FINGERPRINT_UPLOAD 0x08
#define FINGERPRINT_DELETE 0x0C
#define FINGERPRINT_EMPTY 0x0D
#define FINGERPRINT_UPCHAR 0x09
#define FINGERPRINT_UPIMAGE 0x0A
#define FINGERPRINT_READSYSPARAM 0x0F
#define FINGERPRINT_SETPASSWORD 0x12
#define FINGERPRINT_VERIFYPASSWORD 0x13
#define FINGERPRINT_HISPEEDSEARCH 0x1B
#define FINGERPRINT_TEMPLATECOUNT 0x1D
#define FINGERPRINT_AURALEDCONFIG 0x35
#define FINGERPRINT_LEDON 0x50
#define FINGERPRINT_LEDOFF 0x51

#define FINGERPRINT_LED_BREATHING 0x01
#define FINGERPRINT_LED_FLASHING 0x02
#define FINGERPRINT_LED_ON 0x03
#define FINGERPRINT_LED_OFF 0x04
#define FINGERPRINT_LED_GRADUAL_ON 0x05
#define FINGERPRINT_LED_GRADUAL_OFF 0x06
#define FINGERPRINT_LED_RED 0x01
#define FINGERPRINT_LED_BLUE 0x02
#define FINGERPRINT_LED_PURPLE 0x03

#define FINGERPRINT_REG_ADDR_ERR 0x1A
#define FINGERPRINT_WRITE_REG 0x0E

#define FINGERPRINT_BAUD_REG_ADDR 0x4
#define FINERRPRINT_BAUDRATE_9600 0x1
#define FINGERPRINT_BAUDRATE_19200 0x2
#define FINGERPRINT_BAUDRATE_28800 0x3
#define FINGERPRINT_BAUDRATE_38400 0x4
#define FINGERPRINT_BAUDRATE_48000 0x5
#define FINGERPRINT_BAUDRATE_57600 0x6
#define FINGERPRINT_BAUDRATE_67200 0x7
#define FINGERPRINT_BAUDRATE_76800 0x8
#define FINGERPRINT_BAUDRATE_86400 0x9
#define FINGERPRINT_BAUDRATE_96000 0xA
#define FINGERPRINT_BAUDRATE_105600 0xB
#define FINGERPRINT_BAUDRATE_115200 0xC

#define FINGERPRINT_SECURITY_REG_ADDR 0x5
#define FINGERPRINT_SECURITY_LEVEL_1 0x1
#define FINGERPRINT_SECURITY_LEVEL_2 0x2
#define FINGERPRINT_SECURITY_LEVEL_3 0x3
#define FINGERPRINT_SECURITY_LEVEL_4 0x4
#define FINGERPRINT_SECURITY_LEVEL_5 0x5

#define FINGERPRINT_PACKET_REG_ADDR 0x6
#define FINGERPRINT_PACKET_SIZE_32 0x0
#define FINGERPRINT_PACKET_SIZE_64 0x1
#define FINGERPRINT_PACKET_SIZE_128 0x2
#define FINGERPRINT_PACKET_SIZE_256 0x3

#define DEFAULT_TIMEOUT 1000

#define FINGERPRINT_TEMPLATE_SIZE 256

struct Fingerprint_Packet {
    Fingerprint_Packet(uint8_t type, uint16_t len, uint8_t *data){
        this->start_code = FINGERPRINT_STARTCODE;
        this->type = type;
        this->length = len;
        this->addr = 0xFFFFFFFF; 
        uint16_t copy_len = (len > 300) ? 300 : len; 
        memcpy(this->data, data, copy_len);
    }
    
    Fingerprint_Packet() {
        start_code = 0;
        type = 0;
        length = 0;
        addr = 0;
        memset(data, 0, 300);
    }

    uint16_t start_code;
    uint32_t addr;
    uint8_t type;
    uint16_t length;
    uint8_t data[300]; 
};

class Fingerprint {
    public:
        #if defined(__AVR__) || defined(ESP8266)
            Fingerprint(SoftwareSerial *ss, uint32_t password = 0x0);
        #endif
        Fingerprint(HardwareSerial *hs, uint32_t password = 0x0);
        Fingerprint(Stream *s, uint32_t password = 0x0);

        void begin(uint32_t baudrate);
        bool verifyPassword();
        uint8_t getParameters();

        uint8_t getImage();
        uint8_t image2Tz(uint8_t slot = 1);
        uint8_t createModel();

        uint8_t getModel(void);
        uint8_t getModel(uint8_t slot, uint8_t* out_buffer);
        uint8_t downloadImage();
        uint8_t downloadImageToBuffer(uint8_t* out_buffer, size_t maxLen, size_t* outLen);

        uint8_t setPassword(uint32_t password);
        uint8_t LEDCtrl(bool on);
        uint8_t LEDCtrl(uint8_t ctrl, uint8_t speed, uint8_t coloridx, uint8_t count = 0);

        uint8_t setBaudRate(uint8_t baudrate);
        uint8_t setSecurityLevel(uint8_t level);
        uint8_t setPacketSize(uint8_t size);

        void sendPacket(const Fingerprint_Packet &packet);
        uint8_t getReplyPacket(Fingerprint_Packet *packet, uint16_t timeout = DEFAULT_TIMEOUT);

        uint16_t status_reg = 0x0;
        uint16_t system_id = 0x0;
        uint16_t capacity = 64;
        uint16_t security_level = 0;
        uint32_t device_addr = 0xFFFFFFFF;
        uint16_t packet_size = 64;
        uint16_t baud_rate = 57600;

    private:
        uint8_t checkPassword();
        uint8_t WriteReg(uint8_t regAddr, uint8_t regValue);
        uint32_t thePassword;
        uint32_t theAddress;
        uint8_t recvPacket[20];
        Stream *mySerial;
        HardwareSerial *hwSerial;
        #if defined(__AVR__) || defined(ESP8266)
            SoftwareSerial *swSerial;
        #endif
};

#endif // __FINGERPRINT_H__
