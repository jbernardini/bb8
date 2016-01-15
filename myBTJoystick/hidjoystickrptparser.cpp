//#include <SerialHandler.h>
#include "hidjoystickrptparser.h"
#include <SoftwareSerial.h> 

#define RxD1 7
#define TxD1 6
SoftwareSerial BlueToothSerial(RxD1,TxD1); 

JoystickReportParser::JoystickReportParser(JoystickEvents *evt) :
joyEvents(evt),
oldHat(0xDE),
oldButtons(0) {
        for (uint8_t i = 0; i < RPT_GEMEPAD_LEN; i++)
                oldPad[i] = 0xD;
}

void JoystickReportParser::Parse(HID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) {
        bool match = true;

        // Checking if there are changes in report since the method was last called
        for (uint8_t i = 0; i < RPT_GEMEPAD_LEN; i++)
                if (buf[i] != oldPad[i]) {
                        match = false;
                        break;
                }

        // Calling Game Pad event handler
        if (!match && joyEvents) {
                joyEvents->OnGamePadChanged((const GamePadEventData*)buf);

                for (uint8_t i = 0; i < RPT_GEMEPAD_LEN; i++) oldPad[i] = buf[i];
        }

        uint8_t hat = (buf[5] & 0xF);

        // Calling Hat Switch event handler
        if (hat != oldHat && joyEvents) {
                joyEvents->OnHatSwitch(hat);
                oldHat = hat;
        }

        uint16_t buttons = (0x0000 | buf[6]);
        buttons <<= 4;
        buttons |= (buf[5] >> 4);
        uint16_t changes = (buttons ^ oldButtons);

        // Calling Button Event Handler for every button changed
        if (changes) {
                for (uint8_t i = 0; i < 0x0C; i++) {
                        uint16_t mask = (0x0001 << i);

                        if (((mask & changes) > 0) && joyEvents)
                                if ((buttons & mask) > 0)
                                        joyEvents->OnButtonDn(i + 1);
                                else
                                        joyEvents->OnButtonUp(i + 1);
                }
                oldButtons = buttons;
        }
}

void JoystickEvents::OnGamePadChanged(const GamePadEventData *evt) {
        String SendThis = "GPED,";
        BlueToothSerial.begin(38400);
        //BlueToothSerial.print("X1: ");
        //SendThis += "X1: ";
        //BlueToothSerial.print(evt->X);
        SendThis += evt->X;
        //BlueToothSerial.print("\tY1: ");
        SendThis += ",";
        //BlueToothSerial.print(evt->Y);
        SendThis += evt->Y;
        //BlueToothSerial.print("\tX2: ");
        SendThis += ",";
        //BlueToothSerial.print(evt->Z1);
        SendThis += evt->Z1;
        //BlueToothSerial.print("\tY2: ");
        SendThis += ",";
        //BlueToothSerial.print(evt->Z2);
        SendThis += evt->Z2;
        //BlueToothSerial.print("\tRz: ");
        SendThis += ",";
        //BlueToothSerial.print(evt->Rz);
        SendThis += evt->Rz;
        SendThis += "\r\n";
        BlueToothSerial.print(SendThis);
        Serial.print(SendThis);
        /*
        Serial.print("X1: ");
        PrintHex<uint8_t > (evt->X, 0x80);
        Serial.print("\tY1: ");
        PrintHex<uint8_t > (evt->Y, 0x80);
        Serial.print("\tX2: ");
        PrintHex<uint8_t > (evt->Z1, 0x80);
        Serial.print("\tY2: ");
        PrintHex<uint8_t > (evt->Z2, 0x80);
        Serial.print("\tRz: ");
        PrintHex<uint8_t > (evt->Rz, 0x80);
        Serial.println("");
        */
}

void JoystickEvents::OnHatSwitch(uint8_t hat) {
        BlueToothSerial.print("Hat Switch: ");
        BlueToothSerial.print(hat,  HEX);
        Serial.print("Hat Switch: ");
        PrintHex<uint8_t > (hat, 0x80);
        BlueToothSerial.println("");
        Serial.println("");
}

void JoystickEvents::OnButtonUp(uint8_t but_id) {
        BlueToothSerial.print("Up: ");
        Serial.print("Up: ");
        BlueToothSerial.println(but_id, DEC);
        Serial.println(but_id, DEC);
}

void JoystickEvents::OnButtonDn(uint8_t but_id) {
        BlueToothSerial.print("Dn: ");
        Serial.print("Dn: ");
        BlueToothSerial.println(but_id, DEC);
        Serial.println(but_id, DEC);
}
