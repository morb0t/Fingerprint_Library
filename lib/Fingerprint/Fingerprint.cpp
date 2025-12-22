#include "Fingerprint.h"

#define GET_CMD_PACKET(...)\
    uint8_t data[] = {__VA_ARGS__};\
    Fingerprint_Packet packet(FINGERPRINT_COMMANDPACKET, sizeof(data), data);\
    Fingerprint::sendPacket(packet);\
    if(Fingerprint::getReplyPacket(&packet) != FINGERPRINT_OK)\
        return FINGERPRINT_PACKETRECIEVEERR;\
    if(packet.type != FINGERPRINT_ACKPACKET)\
        return FINGERPRINT_PACKETRECIEVEERR;                                    
    
#define SEND_CMD_PACKET(...)\
    GET_CMD_PACKET(__VA_ARGS__)\
    return packet.data[0];

#if defined(__AVR__) || defined(ESP8266)
Fingerprint::Fingerprint(SoftwareSerial *ss, uint32_t password)
{
    hwSerial = NULL;
    mySerial = ss;
    swSerial = mySerial;
    thePassword = password;
    theAddress = 0xFFFFFFFF;
}
#endif

Fingerprint::Fingerprint(HardwareSerial *hs, uint32_t password)
{
    thePassword = password;
    theAddress = 0xFFFFFFFF;

    #if defined(__AVR__) || defined(ESP8266)
        swSerial = NULL;
    #endif
    hwSerial = hs;
    mySerial = hwSerial;
}

Fingerprint::Fingerprint(Stream *s, uint32_t password)
{
    thePassword = password;
    theAddress = 0xFFFFFFFF;

    #if !defined(__AVR__) && !defined(ESP8266)
        hwSerial = NULL;
    #endif
    #if defined(__AVR__) || defined(ESP8266)
        swSerial = NULL;
    #endif

    mySerial = s;
}

void Fingerprint::begin(uint32_t baudrate)
{
    delay(1000);
    if (hwSerial)
        hwSerial->begin(baudrate);
    #if defined(__AVR__) || defined(ESP8266)
        if (swSerial)
            swSerial->begin(baudrate);
    #endif
}

bool Fingerprint::verifyPassword()
{
    return checkPassword() == FINGERPRINT_OK;
}

uint8_t Fingerprint::checkPassword()
{
    GET_CMD_PACKET(FINGERPRINT_VERIFYPASSWORD,
        (uint8_t)(thePassword >> 24),
        (uint8_t)(thePassword >> 16),
        (uint8_t)(thePassword >> 8),
        (uint8_t)(thePassword & 0xFF));
    
    if(packet.data[0] == FINGERPRINT_OK){
        return FINGERPRINT_OK;
    } else {
        return FINGERPRINT_PACKETRECIEVEERR;
    }
}

uint8_t Fingerprint::getParameters()
{
    GET_CMD_PACKET(FINGERPRINT_READSYSPARAM);
    status_reg = ((uint16_t)packet.data[1] << 8) | packet.data[2];
    system_id = ((uint16_t)packet.data[3] << 8) | packet.data[4];
    capacity = ((uint16_t)packet.data[5] << 8) | packet.data[6];
    security_level = ((uint16_t)packet.data[7] << 8) | packet.data[8];
    device_addr = ((uint32_t)packet.data[9] << 24) |
                  ((uint32_t)packet.data[10] << 16) |
                  ((uint32_t)packet.data[11] << 8) |
                  ((uint32_t)packet.data[12]);
    packet_size = ((uint16_t)packet.data[13] << 8) | packet.data[14];
    if(packet_size == 0){
        packet_size = 32;
    }else if(packet_size == 1){
        packet_size = 64;
    }else if(packet_size == 2){
        packet_size = 128;
    }else if(packet_size == 3){
        packet_size = 256;
    }
    baud_rate = (((uint16_t)packet.data[15] << 8) | packet.data[16]) * 9600;
    return packet.data[0];
}

uint8_t Fingerprint::getImage()
{
    SEND_CMD_PACKET(FINGERPRINT_GETIMAGE);
}

uint8_t Fingerprint::image2Tz(uint8_t slot)
{
    SEND_CMD_PACKET(FINGERPRINT_IMAGE2TZ, slot);
}

uint8_t Fingerprint::createModel()
{
    SEND_CMD_PACKET(FINGERPRINT_REGMODEL);
}

uint8_t Fingerprint::LEDCtrl(bool on)
{
    if(on){
        SEND_CMD_PACKET(FINGERPRINT_LEDON);
    }else{
        SEND_CMD_PACKET(FINGERPRINT_LEDOFF);
    }
}

uint8_t Fingerprint::LEDCtrl(uint8_t ctrl, uint8_t speed, uint8_t coloridx, uint8_t count)
{
    SEND_CMD_PACKET(FINGERPRINT_AURALEDCONFIG, ctrl, speed, coloridx, count);
}

uint8_t Fingerprint::setPassword(uint32_t password){
    GET_CMD_PACKET(FINGERPRINT_SETPASSWORD,
        (uint8_t)(password >> 24),
        (uint8_t)(password >> 16),
        (uint8_t)(password >> 8),
        (uint8_t)(password & 0xFF));
    return packet.data[0];
}

uint8_t Fingerprint::WriteReg(uint8_t reg, uint8_t val)
{
    SEND_CMD_PACKET(FINGERPRINT_WRITE_REG, reg, val);
}

uint8_t Fingerprint::setBaudRate(uint8_t baudrate)
{
    return (Fingerprint::WriteReg(FINGERPRINT_BAUD_REG_ADDR, baudrate));
}

uint8_t Fingerprint::setSecurityLevel(uint8_t level)
{
    return (Fingerprint::WriteReg(FINGERPRINT_SECURITY_REG_ADDR, level));
}

uint8_t Fingerprint::setPacketSize(uint8_t size)
{
    return (Fingerprint::WriteReg(FINGERPRINT_PACKET_REG_ADDR, size));
}

uint8_t Fingerprint::getModel(uint8_t slot, uint8_t* out_buffer)
{
    GET_CMD_PACKET(FINGERPRINT_UPCHAR, slot);
    if(packet.data[0] != FINGERPRINT_OK){
        return packet.data[0];
    }

    int bytesRecv = 0;
    while(bytesRecv < FINGERPRINT_TEMPLATE_SIZE*2){
        if (getReplyPacket(&packet, DEFAULT_TIMEOUT) != FINGERPRINT_OK){
            return FINGERPRINT_PACKETRECIEVEERR;
        }
        if (packet.type == FINGERPRINT_DATAPACKET || packet.type == FINGERPRINT_ENDDATAPACKET){
            uint16_t payloadSize = packet.length - 2;
            for(int i = 0; i < payloadSize; i++){
                if(bytesRecv >= FINGERPRINT_TEMPLATE_SIZE*2){
                    break;
                }
                out_buffer[bytesRecv++] = packet.data[i];
            }
        }else {
            return FINGERPRINT_BADPACKET;
        }

        if(packet.type == FINGERPRINT_ENDDATAPACKET){
            break;
        }
    }

    return FINGERPRINT_OK;
}

uint8_t Fingerprint::downloadImage(){
    GET_CMD_PACKET(FINGERPRINT_UPIMAGE);
    if(packet.data[0] != FINGERPRINT_OK){
        return packet.data[0];
    }
    while(true){
        if (getReplyPacket(&packet) != FINGERPRINT_OK){
            return FINGERPRINT_TIMEOUT;
        }

        if(packet.type == FINGERPRINT_DATAPACKET || packet.type == FINGERPRINT_ENDDATAPACKET){
            uint16_t payloadSize = packet.length - 2;

            for(int i = 0; i < payloadSize; i++){
                if(packet.data[i] < 0x10) Serial.print("0");
                Serial.print(packet.data[i], HEX);
            }
        } else {
            return FINGERPRINT_BADPACKET;
        }

        if (packet.type == FINGERPRINT_ENDDATAPACKET){
            Serial.println();
            break;
        }
    }
    return FINGERPRINT_OK;
}

void Fingerprint::sendPacket(const Fingerprint_Packet &packet)
{
    mySerial->write((uint8_t)(packet.start_code >> 8));
    mySerial->write((uint8_t)(packet.start_code & 0xFF));
    mySerial->write((uint8_t)(packet.addr >> 24));
    mySerial->write((uint8_t)(packet.addr >> 16));
    mySerial->write((uint8_t)(packet.addr >> 8));
    mySerial->write((uint8_t)(packet.addr & 0xFF));
    mySerial->write(packet.type);

    uint16_t len = packet.length + 2;
    mySerial->write((uint8_t)(len >> 8));
    mySerial->write((uint8_t)(len & 0xFF));

    uint16_t checksum = ((len)>>8) + ((len)&0xFF) + packet.type;
    for (uint16_t i = 0; i < packet.length; i++) {
        mySerial->write(packet.data[i]);
        checksum += packet.data[i];
    }
    mySerial->write((uint8_t)(checksum >> 8));
    mySerial->write((uint8_t)(checksum & 0xFF));

}

uint8_t Fingerprint::getReplyPacket(Fingerprint_Packet *packet, uint16_t timeout)
{
    uint8_t byte;
    uint16_t idx = 0;
    unsigned long timer = millis();

    while(true){
        while(!mySerial->available()){
            if(millis() - timer > timeout){
                return FINGERPRINT_TIMEOUT;
            }
        }
        byte = mySerial->read();
        switch(idx){
            case 0:
                if(byte != (uint8_t)(FINGERPRINT_STARTCODE >> 8)){
                    continue;
                }
                packet->start_code = (uint16_t)byte << 8;
                break;
            case 1:
                packet->start_code |= byte;
                if(packet->start_code != FINGERPRINT_STARTCODE){
                    return FINGERPRINT_BADPACKET;
                }
                break;
            case 2:
            case 3:
            case 4:
            case 5:
                packet->addr |= (uint32_t)byte << ((5 - idx) * 8);
                break;
            case 6:
                packet->type = byte;
                break;
            case 7:
                packet->length = (uint16_t)byte << 8;
                break;
            case 8:
                packet->length |= byte;
                break;
            default:
                packet->data[idx - 9] = byte;
                if ((idx - 8) == packet->length){
                    return FINGERPRINT_OK;
                }
                break;
        }
        idx++;
        if((idx+9) > sizeof(packet->data)){
            return FINGERPRINT_BADPACKET;
        }
    }
    return FINGERPRINT_BADPACKET;
}