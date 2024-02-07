/*
  New in this version: 
    Hardwarewise - added a voltage divider to measure car battery voltage. 
    Softwarewise - battery voltage is displayed on line 2 and updated every 5-ish second.

  Previous version:
    Uses eeprom to store output pin states. (For some pins I use to drive LED lights.) 
    Implemented "timers" to restart communication if necessary.

  Todo (someone): if possible, enable scrolling text. This should be possible since the text messages is larger than the screen.
  I guess scrolling can be turned on by
  1) editing some bits in the init messages
  2) editing some bits in the text headers
  3) some specific character combination in the text itself
  (Text IS scrolling twice after pressing source/sat display/ and choosing title/artist etc on the HU)

  By Thomas Landahl, 2018-01-03
  Comm sniffing and some code contribution by Vincent Gijsen. Thanks a LOT!

*/

#include <EEPROM.h>
#include <Arduino.h>
#include "melbus.h"

/*
 *      **** SETUP ****
*/

void setup() {
  setupMelbus();
  
}


/*        **************************************************

          MAIN LOOP

          **************************************************
*/


void loop() {
  static byte lastByte = 0;     //used to copy volatile byte to register variable. See below
  static long runOnce = 300000;     //counts down on every received message from HU. Triggers when it is passing 1.
  static long runPeriodically = 100000; //same as runOnce but resets after each countdown.
  static bool powerOn = true;
  static long HWTicks = 0;      //age since last BUSY switch
  static long ComTicks = 0;     //age since last received byte
  static long ConnTicks = 0;    //age since last message to SIRIUS SAT
  static long timeout = 1000000; //should be around 10-20 secs
  static byte matching[listLen];     //Keep track of every matching byte in the commands array

  //these variables are reset every loop
  byte byteCounter = 1;  //keep track of how many bytes is sent in current command
  byte melbus_log[99];  //main init sequence is 61 bytes long...
  bool BUSY = PIND & (1 << MELBUS_BUSY);

  HWTicks++;
  if (powerOn) {
    ComTicks++;
    ConnTicks++;
  } else {
    ComTicks = 1;
    ConnTicks = 1;
    //to avoid a lot of serial.prints when its 0
  }

  //check BUSY line active (active low)
  while (!BUSY) {
    HWTicks = 0;  //reset age

    //Transmission handling here...
    if (byteIsRead) {
      byteIsRead = false;
      lastByte = melbus_ReceivedByte; //copy volatile byte to register variable
      ComTicks = 0; //reset age
      melbus_log[byteCounter - 1] = lastByte;
      //Loop though every command in the array and check for matches. (No, don't go looking for matches now)
      for (byte cmd = 0; cmd < listLen; cmd++) {
        //check if this byte is matching
        if (lastByte == commands[cmd][byteCounter]) {
          matching[cmd]++;
          //check if a complete command is received, and take appropriate action
          if ((matching[cmd] == commands[cmd][0]) && (byteCounter == commands[cmd][0])) {
            byte cnt = 0;
            byte b1 = 0, b2 = 0;
            ConnTicks = 0;  //reset age

            switch (cmd) {

              //0, MRB_1
              case 0:
                //wait for master_id and respond with same
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    if (melbus_ReceivedByte == MASTER_ID) {
                      byteToSend = MASTER_ID;
                      SendByteToMelbus();
                      SendText();
                      break;
                    }
                  }
                }
                //Serial.println("MRB 1");
                break;

              //1, MAIN INIT
              case 1:
                //wait for base_id and respond with response_id
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    if (melbus_ReceivedByte == BASE_ID) {
                      byteToSend = RESPONSE_ID;
                      SendByteToMelbus();
                    }
                    if (melbus_ReceivedByte == CDC_BASE_ID) {
                      byteToSend = CDC_RESPONSE_ID;
                      SendByteToMelbus();
                    }
                  }
                }
                //Serial.println("main init");
                break;

              //2, Secondary INIT
              case 2:
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    if (melbus_ReceivedByte == CDC_BASE_ID) {
                      byteToSend = CDC_RESPONSE_ID;
                      SendByteToMelbus();
                    }
                    if (melbus_ReceivedByte == BASE_ID) {
                      byteToSend = RESPONSE_ID;
                      SendByteToMelbus();
                    }
                  }
                }
                //Serial.println("2nd init");
                break;

              //CMD_1. answer 0x10
              case 3:
                // we read 3 different tuple bytes (0x00 92), (01,3) and (02,5), response is always 0x10;
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    cnt++;
                  }
                  if (cnt == 2) {
                    byteToSend = 0x10;
                    SendByteToMelbus();
                    break;
                  }
                }
                powerOn = true;
                //Serial.println("Cmd 1");
                break;


              //CMD_2. power on?
              case 4:
                // {0xC0, 0x1C, 0x70, 0x02} we respond 0x90;
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    cnt++;
                  }
                  if (cnt == 2) {
                    byteToSend = 0x90;
                    SendByteToMelbus();
                    break;
                  }
                }
                //Serial.println("power on?");
                break;

              //MRB_2
              case 5:
                // {00 1E EC };
                //wait for master_id and respond with same
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    if (melbus_ReceivedByte == MASTER_ID) {
                      byteToSend = MASTER_ID;
                      SendByteToMelbus();
                      SendText();
                      //SendTextI2c();
                      break;
                    }
                  }
                }
                //Serial.println("MRB 2");
                break;

              // CMD_3,  // 6 unknown. Answer: 0x10, 0x80, 0x92
              case 6:
                byteToSend = 0x10;
                SendByteToMelbus();
                byteToSend = 0x80;
                SendByteToMelbus();
                byteToSend = 0x92;
                SendByteToMelbus();
                //Serial.println("cmd 3");
                break;

              //C1_1,    7 respond with c1_init_1
              case 7:
                for (byte i = 0; i < SO_C1_Init_1; i++) {
                  byteToSend = C1_Init_1[i];
                  SendByteToMelbus();
                }
                //Serial.println("C1 1");
                break;

              //C1_2,   8 respond with c1_init_2 (contains text)
              case 8:
                for (byte i = 0; i < SO_C1_Init_2; i++) {
                  byteToSend = C1_Init_2[i];
                  SendByteToMelbus();
                }
                //Serial.println("C1_2");
                break;

              //  C3_0,    9 respond with c3_init_0
              case 9:
                for (byte i = 0; i < SO_C3_Init_0; i++) {
                  byteToSend = C3_Init_0[i];
                  SendByteToMelbus();
                }
                //Serial.println("C3 init 0");
                break;

              //C3_1,    10 respond with c3_init_1
              case 10:
                for (byte i = 0; i < SO_C3_Init_1; i++) {
                  byteToSend = C3_Init_1[i];
                  SendByteToMelbus();
                }
                //Serial.println("C3 init 1");
                break;

              //C3_2,   11 respond with c3_init_2
              case 11:
                for (byte i = 0; i < SO_C3_Init_2; i++) {
                  byteToSend = C3_Init_2[i];
                  SendByteToMelbus();
                }
                //Serial.println("C3 init 2");
                break;

              //   C2_0,    12 get next byte (nn) and respond with 10, 0, nn, 0,0 and 14 of 0x20
              // possibly a text field?
              case 12:
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    byteToSend = 0x10;
                    SendByteToMelbus();
                    byteToSend = 0x00;
                    SendByteToMelbus();
                    byteToSend = melbus_ReceivedByte;
                    SendByteToMelbus();
                    byteToSend = 0x00;
                    SendByteToMelbus();
                    byteToSend = 0x00;
                    SendByteToMelbus();
                    byteToSend = 0x20;
                    for (byte b = 0; b < 14; b++) {
                      SendByteToMelbus();
                    }
                    break;
                  }
                }
                //Serial.print("C2_0");
                //Serial.println(melbus_ReceivedByte, HEX);
                break;

              //C2_1,    13 respond as 12
              case 13:
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    byteToSend = 0x10;
                    SendByteToMelbus();
                    byteToSend = 0x01;
                    SendByteToMelbus();
                    byteToSend = melbus_ReceivedByte;
                    SendByteToMelbus();
                    byteToSend = 0x00;
                    SendByteToMelbus();
                    byteToSend = 0x00;
                    SendByteToMelbus();
                    byteToSend = 0x20;
                    for (byte b = 0; b < 14; b++) {
                      SendByteToMelbus();
                    }
                    break;
                  }
                }
                //Serial.print("C2_1");
                //Serial.println(melbus_ReceivedByte, HEX);
                break;

              //C5_1
              case 14:
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    b1 = melbus_ReceivedByte;
                    break;
                  }
                }
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    b2 = melbus_ReceivedByte;
                    break;
                  }
                }
                byteToSend = 0x10;
                SendByteToMelbus();
                byteToSend = b1;
                SendByteToMelbus();
                byteToSend = b2;
                SendByteToMelbus();
                for (byte b = 0; b < 36; b++) {
                  byteToSend = textLine[b2 - 1][b];
                  SendByteToMelbus();
                }
                //Serial.print("C5_1");
                break;

              //BTN
              case 15:
                //wait for next byte to get CD number
                while (!(PIND & (1 << MELBUS_BUSY))) {
                  if (byteIsRead) {
                    byteIsRead = false;
                    b1 = melbus_ReceivedByte;
                    break;
                  }
                }
                byteToSend = 255; //0x00;  //no idea what to answer
                //but HU is displaying "no artist" for a short while when sending zeroes.
                //Guessing there should be a number here indicating that text will be sent soon.
                SendByteToMelbus();
                byteToSend = 255; //0x00;  //no idea what to answer
                SendByteToMelbus();
                byteToSend = 255; //0x00;  //no idea what to answer
                SendByteToMelbus();

                switch (b1) {
                  //0x1 to 0x6 corresponds to cd buttons 1 to 6 on the HU (650) (SAT 1)
                  //7-13 on SAT 2, and 14-20 on SAT 3
                  //button 1 is always sent (once) when switching to SAT1.
                  case 0x1:
                    //toggleOutput(LEDMISC1); //turn on/off one output pin.
                    //Not used since it will be triggered by setting SAT1
                    break;
                  case 0x2:
                    toggleOutput(LEDLEFT); //turn on/off one output pin.
                    break;
                  case 0x3:
                    toggleOutput(LEDRIGHT); //turn on/off one output pin.
                    break;
                  case 0x4:
                    toggleOutput(LEDR); //turn on/off one output pin.
                    break;
                  case 0x5:
                    toggleOutput(LEDG); //turn on/off one output pin.
                    break;
                  case 0x6:
                    toggleOutput(LEDB); //turn on/off one output pin.
                    break;
                }
                //Serial.print("you pressed CD #");
                //Serial.println(b1);
                break;

              //NXT
              case 16:
                byteToSend = 0x00;  //no idea what to answer
                SendByteToMelbus();
                nextTrack();
                //Serial.println("NXT");
                break;

              //PRV
              case 17:
                byteToSend = 0x00;  //no idea what to answer
                SendByteToMelbus();
                prevTrack();
                //Serial.println("PRV");
                break;

              //SCN
              case 18:
                byteToSend = 0x00;  //no idea what to answer
                SendByteToMelbus();
                play();
                //Serial.println("SCN");
                break;

              //PWR OFF
              case 19:
                byteToSend = 0x00;  //no idea what to answer
                SendByteToMelbus();
                //Serial.println("Power down");
                powerOn = false;
                break;

              //PWR SBY
              case 20:
                byteToSend = 0x00;  //no idea what to answer
                SendByteToMelbus();
                //Serial.println("Power stby");
                powerOn = false;
                break;

              //IGN_OFF
              case 21:
                //Serial.println("Ignition OFF");
                powerOn = false;
                break;

              //
              case 22:
                SendCartridgeInfo();
                break;

              //
              case 23:
                SendTrackInfo();
                break;

              //
              case 24:
                track++;
                fixTrack();
                trackInfo[5] = track;
                nextTrack();
                break;

              //
              case 25:
                track--;
                fixTrack();
                trackInfo[5] = track;
                prevTrack();
                break;

              //
              case 26:
                changeCD();
                break;

              //CDC_PUP
              case 27:
                byteToSend = 0x00;
                SendByteToMelbus();
                trackInfo[1] = startByte;
                trackInfo[8] = startByte;
                break;

              //CDC_PDN
              case 28:
                byteToSend = 0x00;
                SendByteToMelbus();
                trackInfo[1] = stopByte;
                trackInfo[8] = stopByte;
                break;

              //CDC_FFW
              case 29:
                byteToSend = 0x00;
                SendByteToMelbus();
                break;

              //CDC_FRW
              case 30:
                byteToSend = 0x00;
                SendByteToMelbus();
                break;

              //CDC_SCN
              case 31:
                byteToSend = 0x00;
                SendByteToMelbus();
                break;

              //CDC_RND
              case 32:
                byteToSend = 0x00;
                SendByteToMelbus();
                play();
                break;

              //CDC_NU
              case 33:

                break;

            } //end switch
            break;    //bail for loop. (Not meaningful to search more commands if one is already found)
          } //end if command found
        } //end if lastbyte matches
      }  //end for cmd loop
      byteCounter++;
    }  //end if byteisread
    //Update status of BUSY line, so we don't end up in an infinite while-loop.
    BUSY = PIND & (1 << MELBUS_BUSY);
  }


  //Do other stuff here if you want. MELBUS lines are free now. BUSY = IDLE (HIGH)
  //Don't take too much time though, since BUSY might go active anytime, and then we'd better be ready to receive.

  //Printing transmission log (from HU, excluding our answer and things after it)
  //if (ComTicks == 0) {                    //print all messages
  if (ComTicks == 0 && ConnTicks != 0) {    //print unmatched messages (unknown)
    for (byte b = 0; b < byteCounter - 1; b++) {
      Serial.print(melbus_log[b], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  //runOnce is counting down to zero and stays there
  //after that, runPeriodically is counting down over and over...
  if (runOnce >= 1) {
    runOnce--;
  } else if (runPeriodically > 0) runPeriodically--;

  //check if BUSY-line is alive
  if (HWTicks > timeout) {
    Serial.println("BUSY line problem");
    HWTicks = 0;
    //while (1); //maybe do a reset here, after a delay.
  }

  //check if we are receiving any data
  if (ComTicks > timeout) {
    Serial.println("COM failure (check CLK line)");
    ComTicks = 0;
  }

  //check if HU is talking to us specifically, otherwise force it.
  if (ConnTicks > timeout) {
    Serial.println("Lost connection. Re-initializing");
    ConnTicks = 0;
    melbusInitReq();
  }


  //Reset stuff
  melbus_Bitposition = 7;
  for (byte i = 0; i < listLen; i++) {
    matching[i] = 0;
  }

  //send incoming text to HU. (From serial if you have a computer connected)
  while (Serial.available() > 0) {
    Serial.readBytesUntil('\n', customText, 35);
    Serial.print("Sending: ");
    Serial.println((char *)customText);
    //next run, we want to send text!
    reqMaster();
  }

  if ((runOnce == 1) || reqMasterFlag) {
    reqMaster();
    reqMasterFlag = false;
  }

  if (runPeriodically == 0) {
    float battery = getBatV();
    String message = "BAT: " + String(battery, 2) + "V" + '\0';
    runPeriodically = 100000;
    textRow = 2;
    message.getBytes(customText, 36);
    reqMaster();
  }
}



/*
                END LOOP
*/