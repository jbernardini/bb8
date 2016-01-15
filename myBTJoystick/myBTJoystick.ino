#include <hid.h>
#include <hiduniversal.h>
#include <usbhub.h>

#include <SoftwareSerial.h>  
//#include <SerialHandler.h>

//#define RxD 7
//#define TxD 6

//SoftwareSerial BlueToothSerial(RxD,TxD);

// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

#include "hidjoystickrptparser.h"

USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);
JoystickEvents JoyEvents;
JoystickReportParser Joy(&JoyEvents);

void setup() {
        Serial.begin(38400);
        //BlueToothSerial.begin(38400); 
       delay(500);
#if !defined(__MIPSEL__)
        while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
        Serial.println("Start");

        if (Usb.Init() == -1)
                Serial.println("OSC did not start.");

        delay(200);

        if (!Hid.SetReportParser(0, &Joy))
                ErrorMessage<uint8_t > (PSTR("SetReportParser"), 1);
}

void loop() {
        Usb.Task();
        /*
          if(BlueToothSerial.available())
          {
             Serial.print(char(BlueToothSerial.read()));
          }
          if(Serial.available())
          {
            BlueToothSerial.print(char(Serial.read()));
          }
          */
}

