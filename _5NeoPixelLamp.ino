
/**
   test comment
  This is NeoPixelStick control program with the  generic serial control V2 (non-blocking), created for defining and testing a standard method of serial communication that allows both
  1. realtime, quick-changing control, as well as
  2. (slow) transfer of larger amounts of data.

  At the bottom of this sketch is a paste of the MAX/MSP code that goes with this sketch

**************************
***** SERIAL COMMUNICATION
**************************
  The main objectives are:
  - Robust for messages being too short or too long, or invalid
  - ability to send error messages back over serial
  - a Diagnostic on/off function, sending a lot more information back
  - When large amounts of data is sent, for the program not to 'block' but to continue running.
    It does so by not waiting for all Bytes to arrive, but instead only reading the available in the serial buffer,
    then continuing the main loop until more Bytes arrive

  Format:
  (1)"INITIAL READ" : the first three Bytes
  1 the first Byte of any message must be '255'
  2 the second byte is 'Mode' which is used further down in the program for 'cases'
  3 the third Byte is 'DataLength" and says how many bytes will follow: if '0' it means no further Bytes follow
  As a result, messages are always a minimum of THREE Bytes: 1:Begin ("255") 2:Mode 3:DataLength
  - the program waits with reading in Bytes until there are a minimum of 3 Bytes in the buffer.
  - If the first Byte is not '255', it is discarded, and the next Byte is read, until a '255' is found
  - if DataLength > 0, there's a check whether the DataLength doesn't exceed the ReadInBuffer size set by MaxInputSize
  (2)"BULK READ" : Any additional Bytes, of any length
  SerialBulkRead will read all Bytes in the serial buffer into the ReadInBuffer, until:
  (1) there are no more Bytes in the serial buffer: the program will continue it's normal loop until more Bytes arrive. during this time ReadingBulkData = 1
  (2) All expected Bytes have arrived (as many as Datalength)
  - Reading will time out if not all the Bytes have arrived before CommsTimeout
  - If too many Bytes are in the serial buffer, the program assumes these belong to the next message which will start being read in the next loop

  Error scenarios:
  Short Messages:
  - first Byte is not 255 => Byte is discarded, loop starts again at the top with the next Byte
  - less then 3 Bytes received => program does nothing until at least 3 Bytes in buffer
  - More thant 3 Bytes are received => only the first 3 are read, following Bytes are treated as a new message
  - DataLength (intended amount of Bytes to send) exceeds the input buffer: all bytes in Serial buffer are dumped, BulkRead does not get started
  Bulk Messages:
  - less Bytes than DataLength after waiting for CommsTimeout ms: BulkRead aborted, ready for new data. ReadInBuffer may be INVALID if it was only partly overwritten
  - more Bytes than DataLength: the ReadInBuffer is filled until DataLenght, subsequent Bytes are considered to be the next incoming message

  Feedback to Host program:
  data prepended "[" (char 91) : Arduino comment
  data prepended "]" (char 93) : Bytes available in Buffer
  data prepended "{" (char 123): Array output
  data prepended "{" (char 125): Clear Array output


  2 kinds of Modes:
  0-99 are "OnceModes": they make the Arduino 'do' something once, like:
     (a) Do things like switch all LEDs on and off (1=Off, 2=White, 3=Red. 4=Green, 5=Blue
     (b) Mode 10-98 is to activate presets 10-98
     (c) Mode 9 sets the variable preset StateX, based on incoming bytes
     (d) Mode 99 is for changing the Diagnostic settings
  100-199  Continuous modes (ContMode): Modes that have time-based effects and need to be visited every loop. until the mode is changed, we want the program to keep executing this mode.

**/
//Hardware: PINS
const byte ArduinoLedPin =  13  ;   // NOT the NeoPixel data pin!! the number of the Arduino LED pin - it's blinking helps seeing if the program runs
const byte IRBusyLedPin =  12  ;   // NOT the NeoPixel data pin!! the number of the Arduino LED pin - it's blinking helps seeing if the program runs
const byte IRrecv_PIN = 11; // Remote control receive pin. using TSOP4838, with the sensor facing you, legs down, left to right: Signal out (connect to pin 10), GND, IN (3.3V)
#define NeoPixel_PIN 10 //String of NeoPixels connected here


//LED SETUP
#include <Adafruit_NeoPixel.h>
#include <IRremote.h> // make sure the Robot IRRemote  from the Java folder (inside max.app) has been removed

const  int nLEDs = 5; // Number of RGB LEDs:
Adafruit_NeoPixel strip = Adafruit_NeoPixel(nLEDs, NeoPixel_PIN, NEO_RGB + NEO_KHZ800);

//PROGRAM CONTROL
unsigned long LoopStartMillis = 0;  // start time current main loop
unsigned long CommsTimeout = 1000;    // How long to wait for expected bytes to arrive

//IRRemote
IRrecv irrecv(IRrecv_PIN);
decode_results results;
byte IRModeState = 0;
byte PrevIRModeState = 0; // To carry over the previous mode to be able to iterate through table
unsigned long LastIRReceived = 0;
unsigned long IRMuteTime = 800; //once a button click is read, it's ignored for ButtenClickTimer ms
byte IRModeModeTable[11] = { 30, 31, 32, 33, 34, 40, 2, 3, 4, 5, 1 }; // determines which presets, and in what order are cycled


//DIAGNOSTIC TOOLS
byte Diagnostic = 0;                // switches on all kinds of diagnostic feedback from various locations in the program
unsigned long Slowdown = 5;         // Delay value (ms) added to each loop, only in 'Diagnostic' mode to allow inspecting the data coming back over serial
byte LoopIteration = 0;             // to track loop iterations
unsigned long PrevLoopMillis = 0;  // start time previous main loop, allows calculating how long loops take to finish
int LooptimeDiag = 0;              // minimal feedback for checking efficiency: only feeds back looptime
int ArrayDiag = 0;                 // if switched on, prints all arrays every cycle                 // Delay value (ms) added to each loop, only in 'Diagnostic' mode to allow inspecting the data coming back over serial
byte LoopBlinkOn = 0;             //Loopblinker shows a blink every loop. Which can be helpful, but also annoying
byte  IRDiag = 1;                 //feedback what the IR sensor is seeing. Or not
unsigned long msTable[15] = {0, 1, 2, 5, 10, 20, 50, 100, 200, 300, 500, 1000, 2000, 3000, 5000}; //Delay values in ms to 'Slow Down' program for diagnostic purposes

// SERIAL Comms- required for the core functionality of the Serial communication
const int MaxInputSize = 36;     // the maximum length of input data the sketch accepts. Keep it to the minimum required to save memory
byte ReadInBuffer[MaxInputSize]; // Buffer to read data in to
byte ReadInBufferValid = 0;     // flag to be set when full valid data is read
byte PrevBytesInBuffer = 0;     // previous number of unread Bytes waiting in the serial buffer
byte BytesInBuffer = 0;             // current number of unread Bytes waiting in the serial buffer
int NextReadIndex = 0;
unsigned long ReadStartMillis = 0;
byte ReadingBulkData = 0; //When reading is in progress - can take several loops for long inputs
byte DiscardedBytes = 0;            // number of Bytes discarded (read out of the serial buffer but not used) since the last start of a read operation. Indicates something is wrong
byte ReadRuns = 0; // if not all data is read in in one single main loop iteration, 'ReadRuns' keeps track of how many times were required to read all data
byte Mode = 0; // Mode for the program - only updated when data is validated.
byte TempMode = 0; // temporary storage for 'Mode' until data is validated
byte DataLength = 0;

// Dynamic modes
unsigned long    LastDynamicModeAction = 0; // to store the last time an dynamic mode action was performed, allowing timed effects
unsigned long DynamicModeStep; // to keep track of steps in dynamic modes. Not a byte to allow more than 255 steps
unsigned long randomwait = 0;

// PRESETS
// best RGB for the balloons: { 211, 0, 0, 211, 0, 0, 211, 0, 0, 211, 0, 0, 255, 255, 0, 255, 255, 0, 255, 255, 0, 255, 255, 0, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255, }
byte STATE20[nLEDs * 3] = { 95, 0, 114, 200, 0, 255, 200, 0, 255, 200, 0, 255, 200, 0, 255, };
byte STATE21[nLEDs * 3] = { 246, 0, 122, 246, 0, 122, 246, 0, 122, 246, 0, 122, 246, 0, 122, };
byte STATE22[nLEDs * 3] = { 255, 0, 49, 255, 0, 49, 255, 0, 49, 255, 0, 49, 255, 0, 49,  };
byte STATE23[nLEDs * 3] = { 0, 203, 99, 0, 203, 99, 0, 203, 99, 0, 203, 99, 0, 203, 99,};
byte STATE24[nLEDs * 3] = { 202, 47, 0, 202, 47, 0, 202, 47, 0, 202, 47, 0, 202, 47, 0, };
byte COLORSARRAY[6 * 3] = { 255, 0, 49, 255, 0, 133, 180, 0, 180, 0, 203, 99, 202, 47, 0, 200, 200, 200 };

//**********************************************************
//*************      S E T U P       ***********************
//**********************************************************
void setup() {
  pinMode(ArduinoLedPin, OUTPUT);
  Serial.begin(115200);
  // Serial.setTimeout(2000); // Optional, wait longer for all data to arrive

  irrecv.enableIRIn(); // Start the IRreceiver

  Serial.println("Hello.");
  Serial.println(F("[ ERROR: none - initialised OK"));
  memset(ReadInBuffer, 0, MaxInputSize); // Initialise ReadInBuffer, with all zeroes. Valid flag is already set to 0

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  Serial.println("]0"); // start value for host program: 0 bytes in buffer

  IRModeState = 30;
  Mode = 30;
}
//**********************************************************
//  *************         M A I N       **********************
//**********************************************************
void loop()
{

  //  Start of loop housekeeping, and feedback
  ++LoopIteration;
  if (LoopBlinkOn) {
    LoopBlink(LoopIteration);
  }
  PrevLoopMillis = LoopStartMillis;
  LoopStartMillis = millis();
  BytesInBuffer = Serial.available();

  FeedbackToHost();
  delay(Slowdown);
  // End of op housekeeping, and feedback

  //Serial handling
  ReadIRRemote();
  SerialReadInitial();
  if (ReadingBulkData) {
    SerialReadBulkData();
  }


  //  *************         MODES       **********************
  /* Action! When building new cases don't forget:
      Check BulkData for validity before using it
      if the mode doesn't require timing but is 'set and forget' (OnceMode), end the case with Mode = 0
      DO NOT FORGET to add the "break;" at the end
  */

  switch (Mode) {
    case 99: {
        SetDiagnostic();
        Mode = 0;
        break;
      }
    case 98: {// all the same color
        int r = ReadInBuffer[0];
        int g = ReadInBuffer[1];
        int b = ReadInBuffer[2];

        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, strip.Color(r, g, b)); // Erase pixel, but don't refresh!
        }   strip.show();              // Refresh LED states
        Mode = 0;
        break;

      }
      
    case 0: { //do nothing
        // THIS IS A SIMPLE TEST RUN, UNCOMMENT TO HAVE ALL LEDS (up to 60) blinking
        //if (LoopIteration % 2)
        //        {
        //          for (int i = 0; i < 60; i++) {
        //            strip.setPixelColor(i, 255); // Erase pixel, but don't refresh!
        //          }
        //        }
        //        else
        //        {
        //          for (int i = 0; i < 60; i++) {
        //            strip.setPixelColor(i, 0); // Erase pixel, but don't refresh!
        //          }
        //        }
        //        delay(200);
        //        strip.show();
        break;
      }
      
    case 1: {// All off
        //one-off mode: do something, then set mode to '0' so no further changes
        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, 0); // Erase pixel, but don't refresh!
        }
        strip.show();              // Refresh LED states
        Mode = 0;
        break;
      }

    case 2: {// All White
        //one-off mode: do something, then set mode to '0' so no further changes
        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, strip.Color(255, 255, 255));
        }
        strip.show();              // Refresh LED states
        Mode = 0;
        break;
      }

    case 3: {// All Red
        //one-off mode: do something, then set mode to '0' so no further changes
        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, strip.Color(255, 0, 0));
        }
        strip.show();              // Refresh LED states
        Mode = 0;
        break;
      }

    case 4: {// All Green
        //one-off mode: do something, then set mode to '0' so no further changes
        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, strip.Color(0, 255, 0));
        }
        strip.show();              // Refresh LED states
        Mode = 0;
        break;
      }

    case 5: {// All Blue
        //one-off mode: do something, then set mode to '0' so no further changes
        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, strip.Color(0, 0, 255));
        }
        strip.show();              // Refresh LED states
        Mode = 0;
        break;
      }

    case 8: {// Every LED different

        //one-off mode: so something, then set mode to '0' so no further changes
        for (int i = 0; i < 12; i++) {
          strip.setPixelColor(i, strip.Color((i % 2) * 255, i * 10, (255 - i * 20)));
        }
        strip.show();              // Refresh LED states
        Mode = 0;
        break;
      }

    case 9: {// blink in 4's

        //one-off mode: so something, then set mode to '0' so no further changes
        for (int i = 0; i < 12; i++) {
          strip.setPixelColor(i, strip.Color((-255 + (LoopIteration % 516)), (516 - (LoopIteration % 516)), 0));
        }
        strip.show();              // Refresh LED states
        //        Mode = 0;
        break;
      }
    
    case 6: { //manual every LED different


        SetBunchOfLeds( ReadInBuffer[0], ReadInBuffer[1],  ReadInBuffer[2],  ReadInBuffer[3], ReadInBuffer[4], ReadInBuffer[5]);
        //        SetBunchOfLeds( 1, 6,  2,  0, 150, 0);


        //        StartLed = 4;
        //        for ( i = StartLed; i < (StartLed + NrLeds); i++) {
        //          strip.setPixelColor(i * skip, strip.Color(150, 150, 0));
        //        }
        //
        //        StartLed = 8;
        //        for ( i = StartLed; i < (StartLed + NrLeds); i++) {
        //          strip.setPixelColor(i, strip.Color(0, 0, 150));
        //        }
        strip.show();
        Mode = 0;
        break;
      }


    case 40: {
        rainbow(20);
        break;
      }
    case 41: {
        rainbowCycle(20);
        break;
      }

    case 11: {// 12 RGB values

        int r = 0;
        int g = 0;
        int b = 0 ;
        //one-off mode: so something, then set mode to '0' so no further changes
        for (int i = 0; i < strip.numPixels(); i++) {
          r = ReadInBuffer[i * 3];
          g = ReadInBuffer[(i * 3) + 1];
          b = ReadInBuffer[(i * 3) + 2];
          strip.setPixelColor(i, strip.Color(r, g, b)); // Set new pixel 'on'
        }
        strip.show();              // Refresh LED states
        Mode = 0;
        break;
      }

    case 12: { //manual every LED
        SetBunchOfLeds( ReadInBuffer[0], ReadInBuffer[1],  ReadInBuffer[2],  ReadInBuffer[3], ReadInBuffer[4], ReadInBuffer[5]);
        strip.show();
        break;
      }

    case 20: //preprogrammed config
      {
        ArrayToPixels(STATE20);
        Mode = 0;
        break;
      }
    case 21: //preprogrammed config
      {
        ArrayToPixels(STATE21);
        Mode = 0;
        break;
      }
    case 22: //preprogrammed config
      {
        ArrayToPixels(STATE22);
        Mode = 0;
        break;
      }
    case 23: //preprogrammed config
      {
        ArrayToPixels(STATE23);
        Mode = 0;
        break;
      }
    case 24: //preprogrammed config
      {
        ArrayToPixels(STATE24);
        Mode = 0;
        break;
      }

    case 25: //random blink
      {
        RandomBlink(0, 30);
        break;
      }


    case 30: {// all the same color, from COLORSARRAY
        int r = COLORSARRAY[0];
        int g = COLORSARRAY[1];
        int b = COLORSARRAY[2];

        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, strip.Color(r, g, b)); // Erase pixel, but don't refresh!
        }   strip.show();              // Refresh LED states
        Mode = 0;
        break;
      }

    case 31: {// all the same color, from COLORSARRAY
        int r = COLORSARRAY[3];
        int g = COLORSARRAY[4];
        int b = COLORSARRAY[5];

        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, strip.Color(r, g, b)); // Erase pixel, but don't refresh!
        }   strip.show();              // Refresh LED states
        Mode = 0;
        break;
      }

    case 32: {// all the same color, from COLORSARRAY
        int r = COLORSARRAY[6];
        int g = COLORSARRAY[7];
        int b = COLORSARRAY[8];

        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, strip.Color(r, g, b)); // Erase pixel, but don't refresh!
        }   strip.show();              // Refresh LED states
        Mode = 0;
        break;
      }

    case 33: {// all the same color, from COLORSARRAY
        int r = COLORSARRAY[9];
        int g = COLORSARRAY[10];
        int b = COLORSARRAY[11];

        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, strip.Color(r, g, b)); // Erase pixel, but don't refresh!
        }   strip.show();              // Refresh LED states
        Mode = 0;
        break;
      }

    case 34: {// all the same color, from COLORSARRAY
        int r = COLORSARRAY[12];
        int g = COLORSARRAY[13];
        int b = COLORSARRAY[14];

        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, strip.Color(r, g, b)); // Erase pixel, but don't refresh!
        }   strip.show();              // Refresh LED states
        Mode = 0;
        break;
      }

    case 35: {// all the same color, from COLORSARRAY
        int r = COLORSARRAY[16];
        int g = COLORSARRAY[17];
        int b = COLORSARRAY[18];

        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, strip.Color(r, g, b)); // Erase pixel, but don't refresh!
        }   strip.show();              // Refresh LED states
        Mode = 0;
        break;

      }

    case 200:
      { // CHECK validity of ReadInBuffer flag
        // do something continuously, do NOT change the Mode so it keeps running through this cycle}
        break;
      }
  } //END modes


}

//**********************************************************
//*************        F U N C T I O N S       *************
//**********************************************************

void SerialReadInitial() { //SerialReadInitial checks if data is available, if it is valid, reads to first three Bytes and decides whether a subsequent BulkRead is required
  if (ReadingBulkData) {
    // ******* Existing Bulk Read cycle *****
    if ((LoopStartMillis - ReadStartMillis) > CommsTimeout) { //Reading Cycle Timed Out
      Serial.print(F("[ ERROR: Timeout, ReadInBuffer may be invalid. Waited "));
      Serial.print(CommsTimeout);
      Serial.print(F(" for incoming Bytes. Expected: "));
      Serial.print(DataLength);
      Serial.print(F(" Received: "));
      Serial.println((NextReadIndex)); //this is the next Index that would be read, so not the actual latest read index. but indexing starts 0 so NextReadIndex = #Bytes read)
      ReadingBulkData = 0;
      ReadInBufferValid = 0;
    } // end of Reading Cycle Timed Out
  } //End Existing read cycle

  if (!ReadingBulkData) {
    //  ****** New reading cycle ******
    if (BytesInBuffer > 2)  {// there are 3 or more Bytes in Buffer
      if (Serial.read() == 255) {//And first is 255, ready to start reading
        ReadStartMillis = millis();
        DiscardedBytes = 0;
        TempMode = Serial.read(); // Second Byte = Mode, stored temporarily
        DataLength = int(Serial.read()); // Third Byte is DataLength: #Bytes to follow
        if (Diagnostic) {
          Serial.print(F("[ INFO: the 3 initial Bytes have been read. TempMode: "));
          Serial.print(TempMode);
          Serial.print(F(" Datalength: "));
          Serial.println(DataLength);
        }

        // SHORT MESSAGE
        if ((DataLength == 0)) {//short 3-Byte message only
          Mode = TempMode;
          if (Diagnostic) {
            Serial.println(F("[ INFO: short message only, reading complete "));
            Serial.print(F("[ Mode: "));
            Serial.println(Mode);
          }
        } // End first byte = 255

        //TOO LONG MESSAGE
        if ((DataLength > 0) && (DataLength > MaxInputSize)) {
          ReadingBulkData = 0;
          Serial.print(F("[ ERROR: intending to send DataLength =   "));
          Serial.print(DataLength);
          Serial.print(F(" bytes of data, but MaxInputSize =  "));
          Serial.print(MaxInputSize);
          Serial.println(F(". Dumping data. "));
          byte dump[DataLength];
          Serial.readBytes(dump, DataLength); // will read the bytes and dump until timeout, or DataLength bytes are  being read
        } // End DataLength > MaxInputSize

        // LONG MESSAGE
        if ((DataLength > 0) && (DataLength <= MaxInputSize)) { // DataLengh >0 and within limit
          ReadingBulkData = 1;
          NextReadIndex = 0;
          ReadRuns = 0;
          if (Diagnostic) {
            Serial.println(F("[ Read prechecks: going into bulk read mode"));
          }
        } // End long message
      } // End 'first Byte == 255

      else { // more than 3 Bytes, but the first isn't 255. It's now taken out of the buffer so in the next loop, the second Byte will be evaluated. Error feedback to host:
        ++DiscardedBytes;
        Serial.print(F("[ ERROR: first Byte is not '255', Bytes Discarded: "));
        Serial.println(DiscardedBytes);
      } // end 'first byte isnt 255'
    } // end '>2 Bytes'
  } // End of"New reading cycle"
} //End SerialReadInitial


void SerialReadBulkData() { //SerialReadBulkData is only called when SerialReadInitial sets ReadingBulkData to '1'. It is called in every loop until all DataLenght bytes are read
  ReadRuns++;

  while ( ((Serial.available() > 0) && (NextReadIndex < DataLength)) ) {
    ReadInBuffer[NextReadIndex] = Serial.read();
    NextReadIndex++;
  }
  if (NextReadIndex == DataLength) { // valid data
    ReadingBulkData = 0; // reset 'Reading Data' flag, ready for new data
    Mode = TempMode ;

    ReadInBufferValid = 1;
    if (Diagnostic) {
      Serial.println(F("[ Done Bulk reading. Mode Set"));
      Serial.print(F("[ Mode: "));
      Serial.println(Mode);;
    }
  }// end valid data
}


void LoopBlink(int Loop) // Blink ArduinoLED to show program is running. Toggles On/Off every loop
{
  if (Loop % 2)
  {
    digitalWrite(ArduinoLedPin, HIGH);
  }
  else
  {
    digitalWrite(ArduinoLedPin, LOW);
  }
}// End LoopBlink


void FeedbackToHost()
{
  //  if (PrevBytesInBuffer != BytesInBuffer) {//Only update BytesInBuffer value to host if there has been a change
  Serial.print("]");
  Serial.println(BytesInBuffer);
  //  }

  if ((Diagnostic) && (!LooptimeDiag) && (!IRDiag)) {// Only give full diagnostic feedback if LooptimeDiag is disabled
    Serial.print(F("[ **** BEGIN start-of-loop feedback for loop#: "));
    Serial.println(LoopIteration); Serial.print(F("[ Previous looptime: "));
    Serial.println((LoopStartMillis) - (PrevLoopMillis));
    Serial.print(F("[ LoopStartMillis: "));
    Serial.println(LoopStartMillis);
    Serial.print(F("[ ReadStartMillis: "));
    Serial.println(ReadStartMillis);
    Serial.print(F("[ Slowdown: "));
    Serial.println(Slowdown);
    Serial.print(F("[ Bytes in Buffer: "));
    Serial.println(BytesInBuffer);
    Serial.print(F("[Bytes Discarded: "));
    Serial.println(DiscardedBytes);
    Serial.print(F("[ TempMode: "));
    Serial.print(TempMode);
    Serial.print(F(", IRMode: "));
    Serial.print(IRModeState);
    Serial.print(F(", Mode: "));
    Serial.println(Mode);
    Serial.print(F("[ IRModeState: "));
    Serial.println(IRModeState);
    Serial.print(F("[ DataLength: "));
    Serial.println(DataLength);
    Serial.print(F("[ Reading Bulk data in progress: "));
    Serial.println(ReadingBulkData);
    Serial.print(F("[ ReadingBulk: "));
    Serial.println(ReadingBulkData);
    Serial.print(F("[ ReadRuns: "));
    Serial.println(ReadRuns);
    Serial.println(F("[ **** END  start-of-loop feedback"));
  } // end of full giagnostic feedback
  if (LooptimeDiag) {// lean feedback of looptime only
    Serial.print(F("[ Looptime: "));
    Serial.println((LoopStartMillis) - (PrevLoopMillis));
  }
  if (ArrayDiag) {// Full ReadInBuffer printout to host
    ArrayToSerial(ReadInBuffer, MaxInputSize);
  } // END FeedbackToHost

}

// PrintArrayToSerial: A diagnostic printing out full arrays to the serial port
void ArrayToSerial(byte Array[], int N) {
  Serial.println("}"); // CLEARS array in Host
  for (int i = 0; i < N ; i++)
  {
    Serial.print("{ ");
    Serial.println(Array[i], DEC);
  }
} // END PrintArrayToSerial

void SetDiagnostic() //Mode 99 is CONFIG. Bytes set: Diagnostic, Delay
{ Serial.println("[ Entering Diagnostic change");
  Diagnostic = ReadInBuffer[0];
  int i = ReadInBuffer[1];
  if (i < 15) {
    Slowdown = msTable[i];
  }
  else
  { Serial.print("[ ERROR: Slowdown value > 15");
  }
  LooptimeDiag = ReadInBuffer[2];
  ArrayDiag = ReadInBuffer[3];
  CommsTimeout = msTable[4];
  LoopBlinkOn = ReadInBuffer[5];
  IRDiag = ReadInBuffer[6];


  Serial.setTimeout(CommsTimeout);
  Serial.print("[ Diagnostic set to: ");
  Serial.println(Diagnostic);
  Serial.print("[ Slowdown set to: ");
  Serial.println(Slowdown);
  Serial.print("[ CommsTimeout set to: ");
  Serial.println(CommsTimeout);
} // END SetDiagnostic

//***** REMOTE CONTROL
void ReadIRRemote() {
  int go = 0;
  if (irrecv.decode(&results)) {
    go = 1;
    if (IRDiag) {
      Serial.print("(");
      Serial.println(results.value, HEX);
    }
    irrecv.resume(); // Receive the next value
  }
  if ( (millis() - LastIRReceived) > IRMuteTime) {
    digitalWrite(IRBusyLedPin, LOW);

    if (go) {
      digitalWrite(IRBusyLedPin, HIGH);
      IRModeState++;
      if (IRModeState > 11) {
        IRModeState = 0;
      }
      Mode = IRModeModeTable[IRModeState];
      if (Diagnostic) {
        Serial.print(F("[ INFO: Mode changed through IR."));
        Serial.println(F("[ IRModeState: "));
        Serial.println(IRModeState);
        Serial.print(F("[ Mode: "));
        Serial.println(Mode);
      }

      LastIRReceived = millis();
    }
  }

  /* THIS CODE WORKS WITH *ANY* Remote, any received IR signal just moves on to the next Mode in "IRModeModeTable'
    int go = 0;
    if (irrecv.decode(&results)) {
    go = 1;
    Serial.print("(");
    Serial.println(results.value, HEX);
    irrecv.resume(); // Receive the next value
    }
    if ( (millis() - LastIRReceived) > IRMuteTime) {
    digitalWrite(IRBusyLedPin, LOW);

    if (go) {
      digitalWrite(IRBusyLedPin, HIGH);
      IRModeState++;
      if (IRModeState > 9) {
        IRModeState = 0;
      }
      Mode = IRModeModeTable[IRModeState];
      LastIRReceived = millis();
    }
    }
    //END 'ANY REMOTE' code
  */


}

// ******* LED FUNCTIONS

void SetBunchOfLeds(int StartLed, int NrLeds, int skip, int r, int g, int b) {
  int i = StartLed;
  for ( i = 0; i < NrLeds; i++) {
    strip.setPixelColor((StartLed + (i * (skip + 1))), strip.Color(r, g, b));
  }
} //end SetBunchOfLeds

// Array to NeoPixels
void ArrayToPixels(byte Array[]) {
  for (int i = 0; i < nLEDs; i++)
  {
    int pix = i;
    int r = Array[i * 3];
    int g = Array[i * 3 + 1];
    int b = Array[i * 3 + 2];
    strip.setPixelColor(pix, strip.Color(r, g, b)); // Set new pixel 'on'
    strip.show();              // Refresh LED states
  }
}


void SetRedBalloon(int red, int green, int blue) {
  strip.setPixelColor(0, strip.Color(red, green, blue));
  strip.setPixelColor(1, strip.Color(red, green, blue));
  strip.setPixelColor(2, strip.Color(red, green, blue));
  strip.setPixelColor(3, strip.Color(red, green, blue));
  strip.show();
}

void SetYellowBalloon(int red, int green, int blue) {
  strip.setPixelColor(4, strip.Color(red, green, blue));
  strip.setPixelColor(5, strip.Color(red, green, blue));
  strip.setPixelColor(6, strip.Color(red, green, blue));
  strip.setPixelColor(7, strip.Color(red, green, blue));
  strip.show();
}

void SetBlueBalloon(int red, int green, int blue) {
  strip.setPixelColor(8, strip.Color(red, green, blue));
  strip.setPixelColor(9, strip.Color(red, green, blue));
  strip.setPixelColor(10, strip.Color(red, green, blue));
  strip.setPixelColor(11, strip.Color(red, green, blue));
  strip.show();
}


void RandomBlink(int long minwait, int long maxwait)
{
  if ((LoopStartMillis - LastDynamicModeAction) > randomwait ) { //is it time to update?
    randomwait = (random(minwait, maxwait) * 100); // determines the NEXT interval
    LastDynamicModeAction = LoopStartMillis;
    DynamicModeStep++;

    if (DynamicModeStep > 255) {
      DynamicModeStep = 0;
    }
    if (DynamicModeStep % 2)
    { //switch a light ON
      switch (random(0, 3)) {

        case 0:
          {
            SetYellowBalloon(255, 0, 0);
            break;
          }
        case 1:
          {
            SetRedBalloon(0, 255, 0);
            break;
          }
        case 2:
          {
            SetRedBalloon(0, 0, 255);
            SetYellowBalloon(0, 0, 255);
            break;
          }
      }
    }
    else // switch a light off - always the first action after a light was switched on
    {
      SetYellowBalloon(0, 0, 0);
      SetRedBalloon(0, 0, 0);
    }


  } //end it's time to update
} // end RandomBlink


// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
    WheelPos -= 170;
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}
void rainbow(uint8_t wait) {
  uint16_t i;

  if ((LoopStartMillis - LastDynamicModeAction) > wait) { //is it time to update?
    LastDynamicModeAction = LoopStartMillis;
    DynamicModeStep++;

    if (DynamicModeStep > 255) {
      DynamicModeStep = 0;
    }
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i + DynamicModeStep) & 255));
    }
    strip.show();
  } //end it's time to update
}// end rainbow

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i;

  if ((LoopStartMillis - LastDynamicModeAction) > wait) { //is it time to update?
    LastDynamicModeAction = LoopStartMillis;
    DynamicModeStep++;

    if (DynamicModeStep > 255) {
      DynamicModeStep = 0;
    }
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + DynamicModeStep) & 255));
    }
    strip.show();
    delay(wait);
  }
}




/* MAX/MSP control program code

  <pre><code>
  ----------begin_max5_patcher----------
  26234.3oc6812baajr2n+8lOEyp8der7FYFLuB.uGmpbrURTs1Vorc1y9bic
  4hRDRBGSQpCeI1dSku62Yld.H.H.3PP.PPY3rqj.A3fY9Mc2S28zS2+w27WN
  5hoeNX9QnGi9Mze4u7Geye4un+H0G7WLW+WN51ge9xwCmqeritb5s2FLYwQm
  .2aQvmWn+7ydM5GCBFcwvK+Xz8lr71vIiCVn+h3Ue3zkKh9TGymd2rf4xVc3
  hvoS9vrfKW.cIFkOfeBhP7T+x0efi7BmANn2a9dyW7kwA52ezK8tgKt7lvIW
  mrU7nCHplwEqZFL2bYxFJYGHY2Mbjt0mdw+yivTwQpO7O+luQ8iSrDvVL85q
  k8QS686CmMY3sAItyu489s.vvkCXbpdHRL+hoALVo.Fz3K9xcAPSbTnb1cEt
  rNbJXDcyJ7K3cXEV5sZBalDPVDL6CASFdAz4h9FyG96Ai9vvEKlEdwxEAq9q
  4F.2f3Jbc7xfoWE8wQedxle7zIWmE6INQfepGc9MSmsH6yh449rQHGMm6EN4
  2CmGFMnf6qIeh90VRKUDy2KlN8teXb3j5m0C6qY8D7cj0isirdjFg0ysAX8b
  ZAVO5Nw5Q6DrdX+CJVuaClOe30Aqw5gykBhTEJHWXV0EligKHaGAToTO9T8p
  nbhtkWu8sg3AWIFwBAOmZG8XzlE8HzcA8D0I54W6fGl0nfmvmsKfmecBdd0M
  3g83MK3I36.3QbpSvys1AOtnYAOpXW.ObcBdhZG7HtMK343tKfGqJf2jfOI+
  1qgc2gN60RsIttB54Zk9pTWsDJNUKHjCbzIGwIGYNjTsTvrrZFcU33feOX17
  znye4ng2cWhONsRS2N7+YptgbiUpQNDgOBG+QyBTZ2.eem3Oc3L43YgbvrbF
  LR+rmXkdS2NcTvrIKCSoPTTWJgNlDsY1JE+k+RvbT+h4kPic4b20imd4GCFk
  70ezz6BlDNIKIQ7sGEb0vkiW7gqlNYw7v+CnWlRUmbt+UCuLnvubrxgOcV3v
  wQiuitdV3noSTchTXs5iidc+lTNidrwSNXzOwjg2kyWVNMKwkBt4b4fb47KF
  NKghljnatX5zwouU72abvUKL29tvISxfhKldWw2bV302Tx28hoxadaYss9Ny
  +vxIvc+fjeawGTp5m94FNdrgALcy+4gSBuc3hfEgvT.wI9lfED2L+xYSGON0
  3EtyumycFIohuL3SgiVbi9EkjXP93g2EQDcT7r7nvqCluH8msX30yS+IqwqK
  +nkWX3R+vhfauarbTj9AR4IrjrjIEUk5yK037zxsdsjoXwiSdq7jbUrzqxVD
  .6nWDPKwxcM6zK.OJP9GA6MPPz18y.iExs8JPteVIj9fRGIL8JZIfpfuoVRH
  M7t.Ig2Jft3MhQqsdo5EkFKxAE8ECbW4FNJM6xHqAT35DnjVouPNuravQADa
  dvhCwddHs+G1Ff7hgSt1ZjzPNVvazVxQRKQNFtX5vBgeZhaTvZg0BgZReIUD
  7ZT6Fyw5E4EahN0icTldelEiaG7UNeKWLdDRpGk7WsCe+FwRtC3ZNMKOn7TY
  PoaaQLtX57ub6ESG2ofILGzy1eSvjvukfo+mogSPTq3ZaMZIG8BGfF3kBRh5
  DjRa2ZZT5cGgYHhKh68tiPBAhyj1JgDxO.9aBhiQuaBBdNOYuV8btHlORv0+
  O4i5pdH4Sm747kOmqS9+us947kzWxmiq+TnyI+o4O7W64rn8HTU64pGr5FSN
  RgGR1jYeNaZOgp83ZfPnvEtioWJa3jOmWF7S9Gp+1Oy3vVbghMuWBR3YZF4q
  F5.YeNaZOVM+b76IOmni+btc7my6dxy4uGdNpCB6Ygb2nmyl1yx2KQI2PJYQ
  9hXdJokp0EH5UJx44Rz+Tqb3ZVQoJOG0twqUxms94bQJXAVJP9djuV0e3pG5
  hrOlEsF1QMJ7W0hRI0JYzdIWUH54ro8hlMDQPBOBFEq8b1zdDS+iEsVHrvkd
  Pm84ro8TnrZEMMACPjn9IU8IYeNaaOX0MYWTs1OPC5lhJXKZO8pzX0PFVo2n
  wCV0vYeNaZOWn8jCYNLY.yMdp+N6yYS64YZOOc+ip6bBMUOcsmyl1SSNyhzj
  iZ3jWu872h1yhmi3.zAxOETtA9Y5IMhyVzXbWCQ.vMo.brhZM4ygsQjgkOGy
  HJElMAUqLHGOY+igAU+DtFpbAL86ZZ6rOWr3NyTLSyrQR8b9.3A2S8p45Att
  sS7bfpeQXghXgp+6ThRUOGyxmSrMOmaDtHhDSxRtjP7y4nEF.zAd5+NmFCVV
  QSd5pgCVJNnDcNEpFCd5+NumSKNCXJEby7VB8qS1d952N2zXokLG+bBirGCQ
  fdkgDbPQOGKh1j4kfSK2wQNDAqiKpkJzK9X5kTyBl48dcV8dYdl+H4yYj.AB
  jgFF5bY5etEhyz7ZuRYdidNgvrVf5MBKuvqTi4a2ywJWxRRCgGGNI3xoKmns
  F1WTj+RK9wxaOeqlmXA29y.OhZ7pBwmWMmwZqGI78K+0YomXErVaiATaUEZL
  JNRi1F+eQ2dfTuyXmndH67.KG1AeGMfJ13FEv8ZIfajZ+wZEDab37EJzJ52a
  ApQzwkvl8OFsV8OlFGJDab1CNJDP.Z7OKbalDIeaQ6WotWjC9rZGxUhyJXqQ
  08I08yG3lOc4rKiFMQcAT5N3nf4KBmDKx32hozy9f2DNZTvjrf7nv4p8WF1o
  aqll219LU0mwVzmE9cl9bNckB5yhNSeVOkissOisnOea3n6lJEnXHd80ayEl
  B6goYeySekqdiabg02TWjt6zNXfMyazN17Fwh9rZ6C6H84b5JkPqQ5F7z1J6
  zkzYv4b5JEfycGZCebc2myHGhaBC51W7husSG4fA6soCQaSBgd+2jL5HMmEF
  HX.K4fvrUAs10imdwvwlPRKNJJJKB1VEla0+AH6rWiREhZ0vAHC6nURmBwVh
  aEO.YTtX.ln0x2qf1whfPlWigfbxXMaaN1XVDK1IisrbOMcTl93ywXZTX83J
  KU3IWoAcpXGqFNUbXO8zlgJnZmJtTQJVo3BPiTsyEW8dBIHr5KJ+wFKbcapS
  mCV3owuc5vMwn0J9Qqe7iwaX7amNeSLdshej5G+vhlF+1ki3DqVOacDbsieX
  O2lF+1kS4DysVwOm5G+3dMM9sKGzIl2Q0nBY+vxqtJXF5GWNdboKFmeDDucZ
  pQgkMYNNCb4TOWQ0Ou+TgXfmJfsoxuKI+DGfMqEGeF4JHFi2Mpzcmjz3qUHi
  jf8HqoH1tRRRo.IISnEZfcV+nnYCNhOpdMRXzvECQs.AYBcFMRM2dRwHDDV1
  OmlwlSune8RHVDz9zW+5m9+swNTiB3H74ARQcVCHRcbMqWoXi+nzvxgijcnl
  mrg4pW9i5KkhSHBFq5DODlY2U2AZGV8R5T3IhES9ky92m9he8rlwnzxW4zQP
  g8fC.pRsHkvuOddYgiJrYk.hl1EIT.QkNtr39iKa+wk8qhiKaYGyj+.Q3piG
  zIRzDoC3D3OwTe4OjhaT+xkn9QhGnwd7+rvP.pgBsGAj8Az5+365la7MTWgP
  .ySeh0DDXyQg7mi5XdVk.6gvZqi01KG94WDNewam9zYiVJE3+xgKlE94Nxgb
  yfobWuDawbYQnBg50xmcxV6.SuQvxChgMNj2qLN4rDrBWqPUYmZe5fe57+pU
  nTxUgwEc.eYoNfup2xkSGCZpH440beI+Qpw+1mU..k9klOh4d99tJwZagbj7
  RFLfs7bePofBlnrTPAq0lDICPO6mO+72bJB8ryew4u9HqBZS5g7rN1nSpdgC
  NspS3XGXmTvz5XFm1Zy33Uy3u3zm+FzaOG8lSeagqgSrhffcPKFfrhdP3uqz
  Cf4vFxApWE0T3fIkWHDRQnDGNSINEOfI3dD8luR8DTRx+rwRCF9TfU1jAj2o
  rfAF61ZhdalDdCSpVEw22m3p8urN8frao8FLz.LtaIslsqrQOvgWJmTyvKjG
  0Xk1X1htrCbzkPD0L5ZxPabmZ.c4sE5xZHQC3ZWz.n1LiQ1c3k5efCuD+5Vz
  .jzTqGhWmCbzEKpaQCPbrGmJ22IzkzZVc2PhFXqPWORmSz.+.Gd0a3esBu0n
  ng58.fsGPWU9uqdQ2ZTz.t0LruojLrRogRo1rWx.3.FFoFrmfxNvgWogw0L7
  ViztT5AN5h8E0L5ZrViTGxcaMkFd8oOG8CCGOd5zIV4nu5zjCd7D.F19e2c0
  hNnYnFG35GEr+UZJnV0JNUE0onppC68IefkSt7l.UzOjvKmZ92T+H0Paiegb
  bKZM4rOlCN6rYNAAQMmFVovVpP8J68U178EWmBqntBlKVgULGWOFU+WLWGtu
  5uHhB2ER+zDlEVcfrrBAklRZ8JETl6uoJFD0KIgkEUMnBe70JbPVT7fRwAk7
  OaK1J5ALaEwUr2XqHh8MakW2lsx+qZ1Jxg7pUdtCbac1JHN.LUcl8Hakamls
  h47UMaE9.lsBmXGvZ6Uqv9f6c1erUhtMaE99JaUYl29zwiaF2GvcXYI0MgWP
  ksdMZ8A3HAvb1EGHPZs8z4+av3wS+zdxGBI2WMyr.bZc1gYAixu.9ua9P.KZ
  S23bwFlBnMzlu4uZJ.RcEt0yT.VvVwHvbqZT6zvSAU.Is2ihLyRa9EbBzVKQ
  0059rB6eOwoUBu1QcENT5fXt78qU.t3ts5JjutMt14Plshthsxi2trUr8MaU
  G2J.5W2rUGzNsRr+VsBumYq7nca1J1W2rUz6IqVwosJaE0yaO6J3tMWE+qat
  J1A8FWR2WKVQ46YtJrmS2luR70MeE+.luh5v2W1VQI6Y9pNtNftecuukdGxb
  UT280pUD+8LWEqayU80crqgcumrVUKaYEQru0ATzwWs5q6fWCKNjCyFl+da0
  J59luh2sCJT9W4Qu1grsUXW+8ksUDm8NeU2d8JN9qa9pCZeAh2aabkotzsG4
  qXc70q95NNKvz6IqW0x1WIIq227Uc70qt2FnEkjiTWN4tgW9QTX9+m0IDzbR
  a24vePpFA9I04u1X9r0jPI4QYWR9FRmsNohJuBR.9M+D4LIzEfhXUsIWMa87
  VEl1z0E7MA0BH54YvB9d3MkMbE9cBj9Nzatb33fO7Fo7n6pYNjFHmC6B4V6H
  TlrIT10YCnb1W2phaPVAxETjCz2KmBcvZxqWufG.e7ZE8.8GmagO.dWkU7Cf
  lb8Bffg94aR9BhUOlGkdQk+zDs6dYTyH+Zg.LIla8PH0irgZhvZOS15hvZOP
  gShEUeDhuUo0HgUOUtkBAX3VRsRvPpWb8R.38JrlI.mYhxpaBlF3txefxqeB
  FAGkVCER1Q2TcT.d1bqkBQxnJrdJnefBqoBIt6uWvcKr1JXta90WA3lqWiEL
  CkL0Yghkfs45sPDBjtlKjU3PVA4qc+MIPOiP8Xsfx9HojpSybyRXOKU5dI3y
  lkxWnh9fwKB3XP4l2IOYscRz+nbFQErrZFUTSt7ZCO6XzR8J3+JcFBuKyPrp
  OCc03oC0JYVzerEyfPQAyUX0Ln2AwL3bkJSRYMRaqQNpxPRoShhC.1LG8u78
  sYRxC2MljzHboHeW.ZMmiVdzOKCY4NYeyQG3uUkO7lESgQ5VISpvyY3F.0BQ
  LxVAY3pBYo05ScRU2vRz5Nr54JGOmOc4rKCRTd6U7Kn0GDRcSVDNIVg4eakD
  vbd3aBGMJXRd.7nv4J8i.s2pDERUGWJpUqGWE.BcwwkRMBqGW7Cnwk21PG5e
  XMtvay3Be3LtHay3hz.iKz6+l7bffwi1f0vV3N6JYI10imdwvwF6qhUvvFyx
  VYCWq6D5qUl.VqtrrIbjlflzQZzMV7tbIsm6Jua3jfwMUcFPrKaEbd.oSRfr
  BaHURPN0dJobiW1g0vIWCcqTN5H6lZIbEbG8VYIDdLpWA6D3cyld2zYwdra.
  0u269av69LK8tuKq269U269La8tOu2698d2u2698d2u2698d2u2698d2u269
  8d2u2698d2u2698d2u2698d2u2698d2+qXu6yr069t2O7texREJuN8tOqF7t
  e5rxT08tOm4xb0tz2wU3ST+E1mHnjdO7WCd3GaoG98v8d3u5d3GaoG98H8d3
  u2C+8d3u2C+8d3u2C+8d3u2C+8d3u2C+8d3u2C+8d3u2C+8d3u2C+8d3u2C+
  eE6gerkd3me+vA+tMU36iqAG7iSeHIptG9gX0uOp8qIe5aJyMa1m93zG.idm
  5uUN0OBlIaFlY8d0u2q98d0u2q98d0u2q98d0u2q98d0u2q98d0u2q98d0u2
  q98d0u2q98d0u2q9e85Ueimz1rW8wth6cAteEpjDE6W+pWgySlp1qCu5G4Pe
  B0w0s2090kq841lPd5cre0crO2ZG6626X+dG626X+dG626X+dG626X+dG626
  X+dG626X+dG626X+dG626X+dG626X+dG6+Uri84V5Xeu6Et0m340LgquAG2w
  v02qOc62w8tOw1.22qOe6uC92mXq+885S39892u2+9892u2+9892u2+9892u
  2+9892u2+9892u2+9892u2+9892u2+9892+qY+6SrMv8ctW3fepT41FIg6a.
  xczA+j9Dtem2G+Dea8wuPz6i+J6i+HXdy93m16h+dW726h+dW726h+dW726h
  +dW726h+dW726h+dW726h+dW726h+dW726h+dW7+0qK9MNRyBW7K7ue3ieNo
  YBheCRtq4bem9bteWzs9BqcqeegzcGbquvV25K5qjt890u2u9890u2u9890u
  2u9890u2u9890u2u9890u2u9890u2u9890u2u9890+qY+5Kr1u9r6cwtesly
  8Ih5vu9BdeV2uy5bepsN2m2Gy96fy8o15b+95oauu868seuu868seuu868se
  uu868seuu868seuu868seuu868seuu868seuu8+Z129Ta8sO+9QL6iErFJl8
  o0gu8484c+ts+8cr1+98Au+N3eeGa8uOuO386cveuC96cveuC96cveuC96cv
  euC96cveuC96cveuC96cveuC96cveuC96cv+WyN32wZG7e+H38w9rlIw6aPx
  c0A+79Luem2K+XWa8xOqOJ9qtW9ifYKhhe2du726k+du726k+du726k+du72
  6k+du726k+du726k+du726k+du726k+du726k+ud8xuwSZV3ke18iv3mPDMS
  X7aPxc0K+8od+Noe8YV6W+9n2eG7qOyZ+52G89azu9fPc2d+526W+d+526W+
  Nre8I890+.2u9jd+5288qOo2u9890u2u9890u2u9890u2u9890u2u92a8qOy
  Z+5e+K58q0TuuAI2Q+5yt2m58uanw0D69+UHEBUzUXy7XZNKNvfQ8DC3kym4
  w6F6Kvrf6BlLZiaBSmYmA7c7UPqPyR66tIwYs41uj0NU4hxFySHLTv3.k0JH
  muCiFNa1vuf9zMgWdC5tvOGLdNZwTzk2HY4SshxvISltRTRUakhrTtElrXl0
  anEamWxIKR5CASRC71lwdMNktlYxqlSev7fEO.Y3eBFg9wkiGKMo8yTzwu9Q
  +zi9gGZ5jxlnj4zJ1JUXcRqWmyHJSPv1Nq4Wzr11L5pwYs4eRMpR1o98gqTq
  b86VjyxKYOkKRqCNIVqCLyY.QJph63Nv0wwwkJWXP92BtmuPjBMycunW2Ar4
  MaQ.kaH9NJwhqhdipnUB0MMowLIjsHX1GfclYMgDCWrb1p8AbfumGgxWyxgX
  SFrv3gee33kASup38VcUeZ7zIWmYJ82vuO89it5omeiTEnho.R8rQSCzBte3
  jj6n22jm8T0pIHRDagDi2dterEQ+hjRwkvcwRxU4hodBpv06D8GC+zBUkWi3
  8BoT3MR2hwrAB4+HBEwKntT9uUKIdIz5TDxsAymOL85nq1UffEnUD+REtsxJ
  vle0VLmqgQ.M8EaR2HJtkUxbdvhcjLttfJBr3lfoQJ+MZTr+WuPE1we6fJmu
  ZgJmsBn77aIfxrKlp8uD1KyBgKw9fIjRsEv79JGvLrh1CXtekCXNaGbIpU3Z
  4sWDLqHiPf69atuuBhtrwNDGRrhcLowa9NJaNjXQ9nvVDgrqqbmbTGd6xa08
  LdR8+kClna3XkjfRhMAqcrZpfVvBaXZKSTLS3XasQw77j2enXlhUz6hFhdmK
  IZnBhG+ffb2HGuNH2cc61j6jupI2IMk3csscG.T5N0FcNuaSmSu2RmW9N7AA
  2ZIJ7wZeE9HbsuPoL2ADieWKUuO2Z0th4iCGUrfA3tUSvfE9yCiwC3F89HdN
  C77DTBARgCfGosQ9vGmL8hrmdrUGgrravbkM7aU1kHutl182x19tkKJavm8d
  YN2aTL1ggOQ+WLtuO7WdTt65CjTzDoTiLd2NG3zIkBYnprUHzZjncZIPVwPg
  aHFJWx.OFmf40M+TwmFyJ6WftL+jv6.jehcujeJZCZy8D4FjdyMd2jlI5rTw
  PUN66jeto0K625ZLO6dJUPKZqZmB21V0fFZe9Dw3csIFakDrZRLlSWWHF9PT
  HF+doPrFsvNBFWq2LG3jp3WYIRBLDsnPygIdw+pZlAmWTihyKpQIRSd3BVZC
  2ud1vQ3Ht.F0mfEJZeWluiNwOS3vekkIP88HQeOhuC1W+8nN9dNL3ujeDNmf
  OcQ7qiqkSvybaSq9HY2g33BoeZOetfSZufXswokvPXvYHlnX+AttDAiUQRJp
  IblcqCRpzgbWOI0gh3IN2PDESWwX6FYkIt1cqEIUrdIUM.Y0sgyFN3J0ZtIQ
  2vqmLUNONN7xOlC0kMw9qEjc9CX999TotRTw.OtmuRWRgfNPNMvTdfl4hGHe
  .rHM82hgWLc1HHaqfIaHxS8gH.CBcIJ2Shdx+wTtziMvyQ9OZpF+hgW9wqmM
  c4jQai1fICZLdd52G+NV+zhtZFKuS6Uh4s0Ocn7rGztBNgWD5Zmeu7OdW4ez
  tJkLqw5z3NTelZYelaYW91vQ2MMbxBCg.GRATLHeGv87x4J3zAoRxNbyUo6O
  MOHPrDDzyvD2sGFbEvfyTsxwTW.G7Z+wJ11wJo6PjZYW18fqGiyg+yJBJC2B
  0jfp.8bxbEG3qvNwW0xzZtt1Rq4rSn.Sjbbm4JVRTPeUWEEbYcF5WWgs8Y2c
  acAQpUBReEK05B6iYN91rt.0o5qKXRllX4uitrsGqLamw4cGpTaW21c2jxx7
  SIQI8Utoju3tGnRsEED9cmYNa0BwkraxW7SIQI8UtojurOl4b1J4KzJKeg6v
  .GT.YzJ8ks8X0VMhb6NTo4vvT.UpSmoOa8RV3CttLtpBChXwooDbm9JbJw36
  AiPYVaXV1b0ydahy5trVKwCs9L4vqOyO75xXsoFGZcZg+gXmlc.1o4GhHM+P
  DoYGhHM6PDoc5N84sRfW0sCihoIr7J8UD+j1goupkU8hZqwIrtiRyTao1ncG
  uhS2FFjcfXikhXK8UjjDaTR6SrQr0nRq2ZfNTeNG+F286y3Cu9b2Yu4H1tUW
  rCutL8vqKi8bN.6zhNDRaqt.jJpQD0Ex5ktPH.3r5p1dgHqGod6zHkvSNR0W
  0YGohcZjhMarfypqZ6QpvZST6NwfzVzooGfcZ1gHRy5PHMy1dMwoCoJG01U+
  HrtimlI1ZhFsCgz5YcxAlEU5NM9.yjJG7AnMU5Ns3vxBEceleXYhhtOSOrLd
  U2mYGZ1Uo60tGZFVo60dGZ5lp609GZJmBqtbvodJzswGZJntUa08NDLaNI8j
  c5qndo7qsW66WaUZ+0Vbn6ncfJC7ZqSp5NLJpzgqk8ZbWpWasCA86PTHdVG5
  h9Ncnds0l5642g50aUD2i8pbDwRMGfWGFI9x1Vfom069tGuCMEYsXdutjnGm
  sH5J2gsd1M0hxouhmZI58v4wBauKo5PxDb82pSmIt5xDbMxDh1gl8fLAWuCu
  v5XK5zcnk0s9X4kmB.6udssKqK1sPk2IUvwm5pHSMLmqz8goFViBXWZGZty9
  7CfnC0qYak3WV0ODjNlCAoCO9xVmvx5PTqCcXcwt1a48ZUOY6lhHTH3LYLcU
  ZiP7FPcbbX9QeVaOQY8QyB6tamsbUBnJgruTWEE4pfjv8QjqtE3fnCISQ3sU
  DrUPkNL1KIAq5ReIAKgs2HXsdIqNzJVB6iOb9NwkQ7RwWk9JQJtLw9fKydbn
  CogqftUmLnpa0KwKU77m9JQpn6euL6sUo2GLoxZvPfgruqe7Us9X09SCV2Ya
  XgC4kcQ.l3frWuaBG4oDGl9JZJgiz8.6E2aqNKe6fXFdJAKouhlRLydAGrO3
  55PzvVuxFeGIhIoHaSekSJhXm8wjm83PGZEdNcqNQp6.qGIEyV5qbRw5sWl8
  1pU3EUeAdHQu5iEwW05CU6ODxcnE3Y1uT4tYtNNUxYJyUoRUS38PB+YKvAVG
  RQGl2VcFxqtXFreRAKYtJU5ZZ+L64tU4qoJKlA6qGwddFPYOHlgYuxLcncGg
  Y8p3rcSaFbpbLYlqRkwIwr8Agp83PGRaFFcqx5C6fXFQJAKouJUVmb+L6sUZ
  yT8rNIFx5jdbZ7Us9P09D8QWRalsY9wu5ZaxAsMYF6c2CyOzsxyY7pG5ElJ+
  hCFpx838vXEucaJQE367StkDItn0GotGhwAj0ApqG6.rSuao1PJK0tTl9JRp
  30XqS4In2+MF33afAotnbI6zAWtHmJxkANTiyKmEdWz3LthpHeeWKgfze10i
  mdwvwZzMXUQdK91qWWVNZwvqgx7wQeyp9q9mv7ltajnRg.cqRJWuqpqeimNb
  zsAymiTk2OmnamWQoI+ZK25c2Rpua4VY9XLFjArIf6FxVMnSUUNHGUgg9hoW
  qpnLllKYk0CtShhz71LxKpL7Pv5QBG6Ov2CSbTkVOHc.jHSzZCtIo7KG53Pt
  kh44D07YdIETrSRBopRePzanvBTmEEmtHVgrEktHIFEUL5LSAjD0o0BqDcqM
  cUdknaCUgNCaek3lRWpLWwN8pSO+WN6ee5KPu4sm8r+IZKnpb1DUETNM3DbL
  UEFHzRVCKygrJWBGHevZHbxqcrfxQEVPU.5TRaFdcvZP2enqk7HUMVR++zWo
  qQZ9xenRn05ZPk5GIdfF6w+ynNnZEvKmtbhtWJxaBkznBHcwjnzZs1HMMKuJ
  nAxU.IqRRHKXwgWN7yuHb9h2N8oyFsLbxzWNTx0+48vhDFLfIfRPFP8V3hDT
  uZDCdUvzeI7yAie8O8Cy+TndM6iZeh.LmoSrLbSI9irADvsFQfkSta3keDEp
  9usXtmtcKzcRpeUJVPEPR1A1TT3hh0XfUiPwBzEnK1BLfXOFbwvIWqF85eW1
  vm3Xroxjw+JazK4TpwQ+cnWb5yQRUnua7vuTgk1rZsIeeWs3NfP2iW1vymlp
  gfZIWRkQtJbbvuGLadlZ81QCu6tDebZ8Ttc3+CTPfci0iPNBgOJtXwczr.kB
  ESSaxxQCmIGNKjikkyfA5m8VUEM0E6vYSVFlRGjntzJDfKL0FTixcvLLNYcI
  7nKjFQb4GM1wD8gSuKXR3jrqbGe6QAWMb43Ee3poRKphJQwjA4d+qFBl1k6M
  i0G6oyBGNdkcMyBGMchpSjBqUebzqSWHt0knzjCF8SLY3c47kAKuJ3lykCxk
  yuX3rD51QhsYZ5zwouU7XYbvUKL29tvISxfhKldWw2bV302Tx28hoxadaYss
  9Ny+fTlp9tePxys3CJsqS+bCGO1vDlt4+7vIg2NbQvhPXJf3DeSPo8ajFhNc
  73TeM3N+dN2YjjJ9xfOENZwMPQoLwc1BKZWYeZIFwNe4EFtzOrH3VoHjEYd.
  4nqnRpncEAyn1OUcTOL3SZZfvwgK9R1B93zqtRZ2ahhLd1x4LvkMZlb.tF5o
  tut1Zt3lYSWd8MYuWgSHkNm.b2Q0E0TeZb4xzonB4IqvB4IYyExSWXuAvPgU
  CJ0lhMW71OwhB3tKb3.wdXkr8BZZKqLm7TUO2HQQZczNaxnvKGtX5rAR5BY2
  3nZrBq1SbsSDWBevFSvJ1Zk3JpoA0A2MhKQOw0gHwEwoojbEsy10gjK2dhqC
  WIWXPy0lPxEFpTY6FwkWOw0AHwEyqwz4h4We5bg6ItNDItvMlNWQMccnyEom
  35.j3hxZLctnz5SmKZOw0gqjqlPmqnltNz4h0Sbc.RbQbZLctH35SmKudyEO
  DotvzFSoqnltNT5xuWk9CRmQzXJcwqOct758h5ArjqlPoqnltNT5Rrmntzn7
  QEg7NEh7IGFqusnkGJPEGCHP76AQ4b7OS80RJrOkI1Qwt3p.JNaLPGuQ4gSJ
  ZGY21fC221y4iN2AQ6DAztusksDcRSrazm8bsrOyp1o1j3aNKGTfnChCsTW4
  AQmkuNoT409U.MOgkPfOthXfIeawIs+Xy1BoSmgfzV5QQEOEwt6s4BgszYdc
  GQZ11k4cldLyxdLsyzioV1iIUih2HXcePwSrbjg6LyEVuBd0lK3tIWMjaN7R
  otBVMzHlZqWM7qtyilJBqCa1HrdiwWNwULvkqrPgrR+5BCvbGu57Tvfwkd5B
  RE6trnv0b6NfZtpSZmuuGmozRS1HDAmnNrjU4LpUNRhkj8pXU20josEU4Dpg
  8cqUD1o4QXGw.OeeeGR6gvXsiaLGe9sFgqUZX+FGfEt3ABIILwu0.XgGT7TH
  UCf8qS.1q4AXpX.SBvtdsG.ColipAvDGm5DfcaAQDbEErOWc1C4sC.SX6B.i
  qS.VzBqx4rRFb6.vbe9t.vj5Df4skL3VgBl5BKxAapVEkAKp0iqLqckA2R.L
  3AupBvh5DfosqHB0tdz7.rvyYG.X2ZcQNR6tHWKAvb7t.vzZ0RiVPFLXWUKg
  sDxtfs0pzAmVP7Ke.QR7RbaM.lCgrcEAXu5U5.mqJO.BjCBq++3UoGnToDC5
  dHaHPfC+tK3fFHCPwJJkX3vqyy.+vOhx.M5+KWxQ+FFEHXvOUlTbHaCNppRB
  2JJS27SmWpW5RxARJhCbi47Ft6.NiS7TZ.43HGrTOWwIlXUQc932x7diCSaB
  jqCFNZ8q0L1vmgqD8zEKWrP1l0dZlhJM5f4gITkRLzUvUzetMRo1XtmPmyBk
  54IHhSzqtyqZZmRkW9ZFoUNcMoU9d7UXF3jCf.tHIVXLq4jX4rGkX46n2tDg
  K2B4UXG+5Td07OFdWMlNtTmQApz5Lk3HFiqyIWJVPtaESIWR0KF3YZPtOjxm
  bqTZ4xoRJZIG9WrJ0FkLI4A242XuuBaKiUvmveE7sNIgMDcqk2bjCsvaWdKj
  hSh9rvIQelikSDBn3GjSuxpIB28ap0yLwgcrH05kZ5uykV8lL6Em9740Y1zC
  RHE0AmquPKTaG4YcZDdVbcxyFgYcJ1UC5uiLpLaXTkX07fqBGuHXVqx+5evy
  9NewvYKjbv0ICrwzu5fA1wamYfq1FntI925j40fWcKlW.42IlWOutvhrGJbn
  kY+DWaWf5ubToXSoITLeUNQUkRTM2XmejthMYddjDIcZSU8pH6wXN0o4XyBt
  KXxHvjLLhJ1CoSUynOJYhhK0NLZkb2w7Oods4IZK8cRMpoUQzVJ+zJ+SGOtx
  AHpzusqiiiKU4PU7.hKQvhKX64KW6pwSGtvp7vJkqCkI23bv41HxhZocAKVN
  aUVbL1EZMmnrTyLkHJq3mqMEk0.NTy2jaJ8obeFqpos8M6EMlbcOgwiPTAo5
  IucBsQxG9hXyFt7l.0Af6xoigDfpryaNlblej3PbUCyATWvmldRsTDLXeYGv
  c8cwI+y5LG5SERq8c0UUBhdOXXPbzI+41LQH3chrnuv9jnu38cUcQTUhhUB+
  jTX6i7sMiosqzvc5KJcERbMpevbzam9lfDIV2Mu+JqRhu6RhW1nTfOVuEIPt
  .snQrarVK4jOfajJzQMVfNTEFM0TqJQSSTJE.l6sQSG2ZYKpSooRzBy3na2n
  etUJK3zIjrrMkmiNqjELxSu4PrU6aaU9ubMhgrGDRAGSAtoNhAaCdxpAd5La
  5dI5KHUL5K3NlnAVKFFDMkZns66dtNOyGwaBR64a6lXVEPc4sASVVhSa8deE
  pxBEAiDOndGpQSQNEKfEA2ZNLSxaczIpyKUzePh9C9pas5dqtYh6l31It+VK
  U8jL4Uf7yLqXnFbqcamf.Uo5saBTf6D6Pim0d382pRoOJbgbY40GW48TQV45
  j3Xp0fBpKjUPeisaUe55Tz3MQs1DzkXADUNTATEvJSOJ1dt3akAlKg7qrmrK
  rECDzGCmLZNZ5UnWNcTv7G+tIn+Nx4Q99ngyBPu6nymbYf9Nu6nGiVbSvWP2
  N7iAp+BYptQnGLZ5CPymJ63poTzT423Dz3vOFnZL0+96niG9PzymhzOvb88P
  PUABMb7XU4QQ1ClfFNYjribE5X7SN+pqjS+O4+VUVPjzDO40AiFfXO4mlEDL
  QRc7jePNgsp0u3g5duj17Q9dnv4nESQCubQ3uObQ.RKOawb3lq9NWZ9N9H8c
  UCHImTnhHx7UPuQJDL3eeB5hgyCFo5fgSj3nZLdwWjTQqZqQQsku5ke0zYnK
  uY3jqUOopced3vqmLc9hvKUuqEJLPCyRVI0QbBgdlbk+vIKmtbNRkAaliNV8
  Ipl7gOFlWjsyvEnajTzHUMr3QPWJ3pqTk4CMvMIP9Axw8ExwgjzYg51+dvru
  fFOc5cCPR0sBGq6Mp2fpap6hAiNA8o.zmFNYg9l2Ma50RZPUC8wff6PAeN3x
  kKfARHz6F7tI4pImWCUYabwZkLvDHDfnXv4qrBDMfq0J2yyN+U+3Y+TthSc2
  8glvGCU11U9es7xxD069Xk6w2jaIgB3i.TsTWj85qbO6Rk6A+UVk6AWXk6Ae
  +rx8rVt05lPSBp5rWi9Q4BBWL7xOl79CmLY5JFkhdplKCckz0OE59GHY8YaR
  5RTTN5J4fqIymYwX9KjKzpHSsA4K8Y2u3OlrU3OqH7e8gXMNKjVe1zqZ+CJk
  yTpuJ0+zYPRsuNF+cNO7cSvCPuY7zOMZ5mlfNVpBF6DoRs2cGn+jittlRz07
  TrCXMO2wXCu1fHpiiwfHvxcvncpi4iUuAx.jZ3+1jC+Gq00cjr2nTx8Do9ji
  +B5JyMUZWdyzOgTlYfBk5hIUxVqFqbfdmbNL.MznH2OJ0t7tfYRkLucnTSaz
  BchY452MgN.8KyjSvR8xmM7KRMLYCjJVd6syQptgjT3cS3P25Gj5s8Qzwe5F
  oF6AyRoKupREdg51yMZONU+HpWsbXIFfRvUspEh91pQybj9dR5ofPo8WHX1P
  NRtQNGDoe4CSNwkmVjEpIYwpbUBePdT4PFe.N4PBSlSOoRkqmQzqQx2zNHb8
  XJQZTfaryV4IhJ+BiBjBcqXKkZEoXiE6P8Vcs.0e8pWPMBmoTeeC6KT4TVEn
  pXMQzYRVrf+DWeqhJsjsUvNF07Hn5fPfkh5LaAfSZZw7LIpkH4LnoiILb72D
  bphW0VBxRDrRJV4i1dgZMHfQAKRb3aBvLQmYiq3zpUnKQio7enjN+L+mX+pL
  EeqzkhrUIb15ehHRqnRlFx6Q1uPLTqisEhoNEAwwirVApSppVIvcQO1d1DA9
  1g43hv7TCuVA2SoQbI.egO2dF482JAJzBsNK83q0rOVa5wFrKdsmYO6OBx1Q
  rKJyfXXr0ZYObOKyd3qmgR2aIOba6x70dt8VxRUcNnrpOmyfaelQisoKCCNQ
  mImFaeelzMxpw3spS2QREyaGRy5Fc5sijl2M5zNaUmtaHilP1pNccHv6qtD5
  bjKDlGqLUadRmvby1VB5636WZFmvGez8lANyTk6raf6b+YfS7Y1Ov87qyP2+
  xgiChN7iCJ5fKHZEBdXbuoQu28sQugp2tQu68sQugz2tQeKFP3a24viJmCI5
  nIxcfuuGCSihO6s9rpuAZEFDHVzUQ.9Vm5H7XcUXj.GQ0jnHEWoS7eonH00a
  fGkRcYqNIB3JkEN7nMRNPwsVyaQPAdUUhOZvznfTHx1kyhLB8gC1TNL9VA+c
  iXg209Xgmr2iF4MR7Ip0Dvi47azwH9Lq4tSDetciTkk3dEwG480eBjoqQ7YT
  4Y2H93cBhOxAEwWwIaxqz+WAZ.yZEMfoXP2Gl6.hIGfTnhvtdMPx1z2CQ0Yg
  T7dwJHUUb1OJWRB9Af3MPhDNLQIPg.W+VDorEBrKZuXMLjtVXNR.wi3oLqvu
  zCl.V3zYUn2jveaXyhTo1lHE5YvAush1F4VImqLeb3n7WVCty1sr1FAUWvcI
  dC74vAWAhZXcxosDP8iSldQhrhRpDhR5rhREcXofAmt006L5rTjrUua4hhFg
  I+7KtNU+jhwNL7I5+hIER.+kGk6ltamZhLVJVbHENvYutjYVRgxxVQIom1aq
  WtQpZbsRUCh8pEpZbt45mJ5M5tDUsvqiRUiOXnpK5z3BG6zUIwm2MoNS8mP8
  UMQd5xupIcaUEKOSh3xuZIvWgaSrVW8mcy1UQBkj9up39zzkDIf6nhDNzkGn
  N780mD.pNcwpRak5z3T038ovovFJfy4zHVwzyO3pnJpyjnoDgvchKbHUtfpv
  nqrzTPRTBZ25ppR0Nu6MPRfTKNJQhDs4KpJZqSiAQQ0SGjXJqtKpJwNxnIrj
  kwho.wL25euoRArPBInhtHjVqAHADYNn2FLeA5km9l27ze5z2jK7VCo0OU5G
  PmjvfUaYklIC8uOlUF3dPt5UOy65nEOI7n8Ikg9jxPeRYnhGMcLG8eBlMMXd
  gGm2jmZ2TqCT3SUgSIcgoQXywfFNvC7rJjrcG1UJyXaheIsVAKhr1AiyuMO8
  0LojAqRkk09zmcmY6Bl9HfNPXH47igUtRlaFq2C7sOjVlXlr+MN22lkyuF8D
  ZgjCwKmNK.MZ3hgpLp0DzUgKliBmftX4UWIUsPcZjVJM5ZzIxOYARJnDosWz
  pSeewy29MC6pWxXz0LOjLO4tkYDAPEGevJQ33TybqH+paaxuhkLrpJlAQpyM
  CwQBjKxC4qNy5Xrp.9go5mQ9jBD1Eg2Q1WZyv9xf8.ApjNQBg8aH1WLyK4zM
  2eWXeIdsE66+Z33PCy4sJN4bXh2wIWQqszp2txpBV6hwd6zRqrVkUktkrpdx
  muaxtlY01Flck5kbJ2krSrq71ZJOZ17nlYJ.BUWCZXT3YsDZQsk8LfhfrYJf
  6l6Ky1YfVKaBEwXcTqrlUqLCfAMO4hcZFfzVKYc0xwiMKMIWi5s+7qO8zirR
  tEu0VIxksqF4YJMwXRzlRUo4Dm1boHtdUlrqD0aYmjl.XyLyoN6jkcDbawmM
  +1gwLZe83dEB0c2cuB1qMY7DnD7cG0dLLM15RTeMmBA1j.CCSUWXB6uGVXpa
  xuv4IRva0k6HMEz0cieoUcuAM2EpV2jotoYRvYaCSRpgXiYlTjSIMITbXGrp
  5RWXQqojdNStGsW02.hbYGmDBzZNOIaRsRl7RXABqscRi2lrltZtvt1rGDKF
  Xi6gaoYuUQp5tL6wZqE+hcsOZdnJCKCkECnlXbTirbF3zNWPNnKNajSUEO.V
  diYKl2p9+yQGWLMj6Hv9IgEJtQ05i.66B7xb2I2AQasc+BldPilFLexCjz9K
  FJ4.9T3hazGgPcp.WNBmphJkNoNglZAiwRJfvu55DhM9OrzVyV1n1iORtXCG
  wEPsPmf1qK1XbFWDF1vK1DoQRzaam35vslK.epzPqYAiZl0ULK.6kvqBUecE
  aZLagWm1Ddmd0UMD7ZJHchUIAzc.dsnwrDdaM2D7ud5KN64MD1ZbbhdusXrc
  DasnwrDaassy9rWoQWziPxmVWY4d4vOe1j6Vt3Mg+m.zSxXU+9Y0V.QwqGr6
  UZSqg8wDuShV3s07y4u5Ym9xye9oMoEAbRB3cWsHn7FyR3sUMBSWwaZWavhb
  OT8XD1FZMa2mj1zJLRI0vicRaPpm+J0G3Mq8W17trVUkVcmGwML7yEsG7W56
  x1kaa0seBi00VohbmdmbCfMN1HRTiIhuZJqrx911oM.VvtGUomJ7jUBodQBg
  TRkcx9xDEwGxjifAXEzb1tXte6UEota3jfwMkYXPJUwT8eMA6rn5Fhkn4Htf
  potUDhcOPvPiF3.6rCDf2J8w2I84ADDN9+TlnhBIR9tTZAlcDIUIzbxpbcRw
  ql4zq65wnp54ny.Li636oOu8DpjiJ0oW2z2ld2zYwGPsAT+1ptb3YaQtPw7Z
  U9S+1vQ2MMbxh4Q1B6osbCDcyodCbitL8as4qyDrFdnRbLAeAGxJslgp5xVd
  nZct725gZKzma5YGpIGXgSM6ntrkmcHMMOmJHXUKZSc0iwnwJbcaOX4M8fkA
  CVmTyq5Ka6gZSSBy4fuD84ojk5yacYoVW2M10gpGIEILbcaOXwM7fkYBUAoB
  EoFr5qa4AK1uoGrP09j5HRRDqursGphFenB4ph8uBA3lVJLkAogGHGcvXrAX
  RdWSYol0or1eVm0RPAkkZAI0ks8Pkz3Z7B5TI7Rsfjvq8mUcbZICYHoYlIaK
  y7825GUQoEqeQmLAsN0XsxEXaYpwJJrpcL4+opkQAgjHH2UeZj8pTtDjSOJw
  3JGmbU24Iddclm3MvHr8n0UFhOuHnQ.9VTXzb2oJ40ItiMY9c4vedvUgiWDL
  qUSH7b6SH77tbBgWWSjJHAhQaz7+sphYnIRf5CfKszT+MsNSdZ75KeoA46Ph
  aQ0oncEkfTgnKGNyiUKWe6fqSviU6fmINZZLviPqN34WqYsOZsiclXboovNg
  Oq5XmWsR2QparybxcZNrCDrUMrSPpSrCW6XmI0DzXXmg7tRXG2uiVlHLXmI7
  zZLrywcGvtZsLijc2xqASDrRU+HcKbcJXysycyqqKc5KPaKzaj5fN45mcSvk
  erRoO7ck7.DGAfwFpzJd76kYaTi2X8zFjH.ygTIg09zMZe5FsOciVsHhJXb5
  bcmkAjFY6ioLseGNwlb3i.3EfxqEM+b3yd477NWuD.Z7vYWGLCRrbBJ5hurv
  xr0ZEh1uZKwH4.BOgfZBSxOf8REuMsSL488HwtdTOsDiBUSH1ASXO.eXah1i
  1ZQt3ou90m+5cLnurNvsXvAW0D.uEDcgoNT1sEPbmjCbA5kO8e+H4X75oxt8
  i.EyVnj91RviIspZPIrvai3SKKg57+4QcCwNXATJQHPgrEuIfxss3mBFK6iH
  mRV7iVHPQq9heVuHH1T3TfM5.16ix.tVaMvOMScZbFGH43t4uhd9vECeg9u+
  A4pfeKUBNnWDNeg4ATfUWekQCyrp3Op05vYSqLRZIZzm7jty5hFOjCkRD9lk
  24gaIPZA5BTXEj6uCJvZCdgMIlJidDNaDvbasEPGd4GQahINLCSbwGFIu1c4
  D.VIXyF1rYBQWmVcEktloTfBJdl.4einkf2Rn02ldcg8sR+PxfofMTXcTpsD
  t8eFOPxB1NzT1oUh47dAVIQ82LAkS6AUSjhpncHvxPV4vVU6EJ2MF0Z9ZPOx
  KDMbZYmOrJYVPi+YwJOjZVLJ5kVEDT0C.ACi1xjQPAJpwmVVfArsACZjifjq
  kAtmtu1QNrJV2m6L8XwAHJacelQO75ycldrJzvwVzic6T8XavXxA2gaKm3ze
  e0kIGfBLHGfBLHGbBLzJNYWeV3bH1q49cndssG9L9AXetKI6P4ZLK60tNcrd
  M15dMsizqc1hkvwci9rvVEkxi9e+0qslWzkdH1qEcH4dBaopoGf8YfpFen0q
  0RZbOz504HqYu0q49V2q6Px8r0FQQM0ku+drNKJj2Ws4d2rH6l68XjX0t5Em
  4XysfEuMQLe8Day9Zej6SxV+WRugdj5Lpt0QSTxPqxlH5dqBlctm93Iyfsrz
  yoz.2l6cTMd9dmELbzWZ7i2KGhsGJ3gaFtZGuWpoDNAoLrbZEaNpbbQ8dV.V
  L8ZUNEKmy2Kbmei79JbP.JBGgCbjAGINVfiktam4dxJb8Sfx47NrBkcs4n81
  bmgWC1i8r3P7lcdZucHdKhG8zYylNC8Z0Aw+wUPDTIkTqURTcq3QtWc1JzTj
  hBZEqnV30n75EnG0Lm7lGU9xSPd6xHCmfKWFdkNryRNiESmTeRSHPbrYn.fB
  RLgscRS1XlBvfKFBjbdIVQfPpy07zqkWibRzjktJANasqcqXjLzO7Jk6JvLQ
  MxHMqDkdbZVU8.8gLzLaPeHVsl3.dy3oeZzzOMAoWwA88H+eC87vgWOYpz.j
  KQRQvnESebbHAmazpVGmiTfjhXN623JkHOJChiH2XFBs0REz1QuUq7kOS94y
  QuM71.4XAc7sye3Q4gRaFxqDWLCNasXHxUMAgAc6YiM4vYGn0HtlK2Rjk3eT
  slEeT1v7zYyF9k4sKlBE+YH+LH36FlhElSlNshXJuQTvmWmJ3Sw.fYxaFUYI
  4MofuvTNiErBdGVgkhtgB9t1qfOl2Y0v+ESmdm5XohtJHXzECu7i6AlTeRsv
  iRn6FOJoQ3QY0OOZT05tIYQM40lJxhR6Frnh6ErnwJFlTujZi4CpDg9zpZDN
  vs4AUrWrW038pVJ3pH.aTrVy0NZwRHpp5nEyamjTYJLg0ktum+pe7repNQJP
  WInrZxc1MnxExlRP9bdqQJGuFQlNs9koybZbY5LwNHR2oiHRmePIReojiaYI
  YfyshPhZEgjqI+yrtmKVDbqI16k25nSTaUr42Dyu4QedzCPh9Cd7WwY08h9K
  pypGKwyk3AS7jIdzslH+jLmskbI1AyegEjDjJkW972uz5YIN1bZ.kVAJ8vEg
  CGu93JumJ53M5jXKu2CJBo7OSSsQnDiyhgBH.ccmfl5bicTMaA16l77ky.RR
  q8tWMLnUG8RH+pmnT3kxBrTtbqtG0+slZxDKWdyc0r450R2bJyy0UhX6oyFs
  LbxTzOlwdZKFi9aWpRNoDwb9+aRq.LgaJDHfEezx23Hh68wr1FjCcf8c1kp8
  +pvi1my15yYa84rsJlBVbcP48+JLQqv5FIZEWHDeHdIRf8hRNPz9M6o88lPy
  Ap8rWidcvkAgRkMQiFtX3QoJpfSltR7SoOZSd5gEfWQLoQoMe7gcK5vCu1.n
  kxZPu6Hm2cTWImTAFvoqkm4UdCVKe+30rDhYnwhTuAR8tneX4UWELC8iKGmp
  jelzTWy2nsHF8XZFXlQY1MRLRJ737WxPskR9fOAI5PIhDOtIFJcSDDjkQY5z
  RhHecvvQxt5Orb7GKgxsfmpQELZBnQelk4UA+hnES14aqj0Wvhi5JYGTSgoD
  aYJvg0RDd5vZ.c9xE2sbA5pYSuEkivt0DfZw2oIIJ4PdNjBwd2lIJYEKernQ
  R6HdT8Jm+kauXZUp1zMRt5yEzCB6.lSt4z+UchTo10hFFJrJIy44OvS6EFOR
  9Af4ZKXHRS5ttuWcZGJqvESGVgLuYCAjFxJhKaUdJpz0cIs0RDXjmJIkhXx+
  uS0+uBMMj1QxAmF9ZHamhorrGAn0lAZs7c5v6tKXxHz+O3i5VPEjPwDaL6nS
  buelhpU9RD1oetc4nZQaQwrkgY79lfR.ECWN3jZNwqUS0vaNUeOcTPqk1y84
  Cb83tTwIQYBdgaWIEvCvw4StLnMgDgTUKAi34QU0VUMX3u4JxP6hIOa5jEsI
  lvUgUDlQ7YRwOvwhxeyphRZULIJpSebqkS74lpEplswey5P05vQqAElSYo0R
  PZqzI6L4.KPCE+sGuh.A8rkylIs37kgiGGN+wZAtONlm5wwRbdLJgKRdLJ0A
  sZcH0uPH0uhoa0x9+aToIUv1533PwmnidNs1kaTLlnV8xQ58oO8LiRmbU3gi
  N9xajFkcozlLD14gnYA2NU4pdss+WI0q3cSlErX4rIodP5CQWMc1kAyQKtI.
  csbZ9NoFFnKBPpMOEoNvMCkpvdA3d0KGGLbVvnsOs7WybIJs7E5HaE1TCu0h
  Mh0lO7Zq4C0ViftaVfR0eI7+ti9s2cD.4He7CkOQjmqMMx6lr1238q9FT02P
  k8EliF96CCGqr0FENw3u6b9t+Q72ESnO7wvg6QMOd2xEa3w4xG+YpY3M8kNN
  9KwbjemYQ6JzYudsc1JE4A2FxiB1V95R9poHFA+hBmGW2Rb6gaJcsKH2.z7x
  eexSJIQ5SxzCyF0BaEr1TNbRZkEU6vIPSPJei0yERm.42RGY1j37l2lbWvW7
  Pzk3leIn5xoigX3QNond.mAd9NXeg5ubL+EtjoEQmXZwZu.tuI7MyIDnbdvD
  MwbRbQ6b+Nm.5KubRvmuSN7CF8iRVmsY+S1YYWUyaPFlFt43Otwrsuqaufos
  TvDgZN49dMmfIORufosQvjYNg.gzRyHXh6zgDLYDE8r0UdeOoLpGVZFi7edt
  wV7i2b8dh2MzHE7HfOUZUixTCk8CRyAJDUE0obdQM4GfMpzpCr4dvjhvei0z
  She+RCaaH0YvXntj2LKM32uzv1YrlYNoIWZvm1IlS9OiAudYsw0VMuzXEQKS
  5iwSylv23NG32MTU0TxTwNx02pWFf1ndpFULtrLvgHs35ysXk5x9pgm0EqKr
  29oXcEGhffiUOKxcpkDXfk7jMpitAAw1GipDmhfzrifVBggMN3zauawWPOBo
  SqrpMXv3y3xPb6+lM5LPjeK2h4.bgyAaXL0NyIpsuC82JA5W+AZzc+MwFrwM
  4YHK.YRQfro22hXYhi9bgvY1moYqw7NC7IdbO+sBQ4khn5AP6.po1U4R.1Be
  tFMPendCnbWrq2VAthh.2zCh1p.M5aa0NjzcpaR91VLe7IGd8YhWmo5P3aa4
  KysCgy11mINGd8YbGpS6YaQ3vUzc5y1BzhNT0n01hNpa2oFd5ZMSn3vqOqUq
  uqTPZYqU.jZ1JgSM0k8rolGd.BycmJ6jtKSOnJWY5tL9fpz+p6xBaDZ32s5y
  ba5ycLRChUBmEGdzyZKt6RcZ6VFriwFxrpSe.BzcmRytsk3PutCLSrs.G5S6
  V8YaHm86Lpan6x1rnhG+vClc6VcYaVRg6zs5y1r1MoiI0vFsmwcGGEPr0StG
  fc4NjBRDq8SPGRrA1RwF911muMbzcSCmrvrKDdPxjWXpu.z35GS52Y6LPIM3
  .UM1TCQHuZEMRYYCuoVXjZqtXjNT8H2Z18tCqClcvIU05tbGRppNDfrqSi6K
  p2UKqWOOX7p5RXtkfvbhGa6JuyjsrfftoxCfp53oNVuBnrCJJKmW6xq2B1bg
  E3GHAS9qyCJM6u2FUjKOnJOqi4.VEKDgpZGotfbUPiXQEXvk0H0sEbcVLt7R
  T9jZl51RDPR2gBXabhWdOW2VXaQcags2qaKEImCoJHF0WMq2thYfqmNO8igf
  je8S0b5baia8JyZi04fmsJ01jeg5s9QjnZCEGpqQqmcaRk9JYk.H2CqnAOxD
  EoPVdA6BgJsuJsl2WTC5KpA8E0fJkxUd5qe9ud1qNG8rye9ocfi0YpyKRN0X
  u0DB1hmDpxfwu6u+2e2DD5s2DJWJcN5UAS+kvOGL9MKBu7inKkcKIIF5tYSu
  VtvN5SgKtQmyfPWGLQphrJ4DpUTN5A+WDzwSlN4QWnDlIAnGdB5xYARBiQpL
  NDRJyQtb8jq0oWnEZS9j+MRx1OYzvYiPRUGtY5HzzqV0t2d6xIgWpk8IeyCW
  fjLRS+zbjjw7FUGGOPEQ8iUrOmf9eWJ60O5xajlcnZYSu5Dzv4nOELdr72pu
  BY.534xF4gnEyFNYtJ37kuwwCmcs7uFdqRE94pOQkVaF7tIpuxSWnG1fz.08
  Vnvq4eLPRGnfsgn6jr8AvcBPu7o+6u6ku4WjcfQAPu95oAyifu3uops+6E9O
  y8Pu4zWe1SegjN+ku7We0YO6ou8ryeU4eO0zY.51ggSPf31veWkJglE7X08d
  D50SuX47E5YDSdoThmAJDSJNCo0.DIum5BklimntPpO2vwgifFX3EgiCWnOE
  CyUI6zfYyR0VCkzNSkKHalFMeojI0xoS9toWcE5pkStTM0dhtc.pgwSWftc5
  LUFOR1CuEl5UMIzL+22DLAlrxNWolHTKTdhdjolGhnamLUkBMQOPSV9.jTMZ
  0kJxivIKCPyVNQQUNP8BPnyVfFolslOEcwWze0OMLTSopZVI4mI0LIaggylI
  g1SzMX3DIAvPIw6jweQeHOz3orSjJENo9.CsMjdsNAdoKTiJSGJ5Kpm.GqBC
  e4nLbL.JlzBk9ECzl+nFjzSsGie36N5rWc1aUDLu9zm972o3wUs0Ugylqngm
  EXZBMuShao9PEPNbxWhlHQ2pnRtH.8.Bm+.Mmio+K6nijfi7aHg7GnR1aO.8
  oaBAdgkyUb6KmIeTIGuJulZF2QyFJX7AWNbdvbciR02TxVHE.7CQs4ykymuH
  Xx0Rl7izRKlO7KyQ2L8SRTQ1CuPiBeJTNYb0Tk.gGiBuB8.mGfBkDOARtZ4D
  Wbm.vL3A0byJFVoRUR0eNYEUqj+PN69I06YHRtFR3sK0r5u8me8omBsgbgiG
  +CARYKnie2QRP4cG8PD4wpwOh93DcYMgZxgrhBxv+GQZnxmW59EL4p5nAPeH
  4KmZdHCDBzLCfWvYWkc9KbtldUOechBJjevnv4WJksFL5DPpq7aLQJ7O9Kn5
  OmX5DCgup5iuRxZY31kH6pwF56U0pRcu8AJfRt7ujY+S2DnQZUqm3YU7QSdv
  BTvmuTkr5T2UkS.Oah4LQoVNVmeakLZub3mOaxcKW7F4mookIRZ4e3Wew+bE
  g7Sky7CGMJTIQXngM7jHh1wwPO37FUVGT8x.pD0nLAuadbhxOTxRmsOZflH1
  qDySRBrDbj40hONMMfpeDKyIbwCTSVRV2wkxiO.MZ4LPhfbVQWW3SjVEUXM5
  IHrAvPOU9JhRDOll4Fopnl1ZD5X4JgZFH4uUeW.zdnYYACkotipeSpLAnb1W
  QSovN0nImF8hfqT8ZUJ1Xt5.CMUk+3hHPkqgnegQioBjBlBoj5sr7VH6DNWR
  yGnVDBYlazztQBn.YN59qTGhYKLqhomrMuF8yOVWPGUcpS0qTM+RoFzyBmNW
  Os9F8BduzHGvrF45bURNCzS993OJAekd9S2AjivqURsGB5KHs7XkNSwbcvKX
  r78Ah8i3wiSodx2RDTnWIR91UZaFwjJGOARsMRJa3BSlAT0vuTMYHU5XxpmP
  A6Iac8ZTqjcPMOfRP.HkT81V8UWXTganhgWZESD9CuuD76GKYgf7EHrzrh2L
  dwR0J7OzHJ.x7jgJ98XdEEE1EQCn2jhyT0IFs716TfcLecDzH0EcAf9AZAVp
  GH6roFrM8DIxjrOO7JUNwL4R7IIjQ2JE5G+JGdwT0a4Dyg3T8vJ3PqnXZIa2
  N7KpUNO6U+qm9hydthIJToGwb.5uS1Yk+RohzmlEtPhYP2LA2elt4iWW3oRH
  sjxWB1.cQzCeibQMooYyC9eWpxlmqlFkxdlGNRJ9ZjIaeFSUFNQpnsZ3GOwp
  UrvTuLUO8OKUaKhnTioaeFuLmuyVjyKy4aWdVubCeghx6kvPmfjVuLRqYoZ0
  cfLx4Q99Zf7cGEkiam+tizyLRclF9Q.PiF2OXzzGH0gTYRiBYmJ+FRIEgeDT
  BW9uiG9PzyUB0j2dt9Nn4RgERAZJNgWb5yUDK50rUZJeL9Ime0URaJex+sxi
  TmfnO40AiFfXO4mj5zIUfl+jeX7xfn19hGp63RCRejumhTQospxP.IiLR6PI
  orJ8Mi9FWZ9F9H88TCkeWJiTOO.eAzaVH+1+ao5tCUp3McxJ5lKhToT0RihZ
  IePQhYnXKxz5GjJ01q35.kQcbdDV9UzYRX4RjSWJWpRAwniixsvODR1vyASp
  zqAoVl5QP+IPRnboRBrDwlDDSjq7WjR3kxugeQKpdvJUtzuAUuT2CUb1eRtp
  hRzYx0ijMzGCBtSJ6J3xkKhWLV8cG.1v8cpe8cembRSZt1a+0e4cS9aRrY7R
  Yi+e8zQCuZ1xvEeHx15A278x6qsEN.8Km8JDVt9sh4bARoCBZhdtWtpN4ef9
  tuC8JcAkVQJ95e5GzzERJn0ZToDvYg2I+Vqcmi0s2Ip2zInWc54e3md8Of9V
  8e8O+4++7bbd3+.58+xqO+md8SeozRyW81We9Kh5SZ87MT0uHXzuHYJehb5h
  hP+C4rspCd9a0vUbWAX7Bm7W+qfHl3APR9CEVIeFk9kJUgtXb3DkGCP2DL9N
  kcb5UxCuJ0DgzPs4o5VRwCy+RbmBSPJDa5cFEDUMubtSxIq7fPvJUu0KwnaT
  o7t2MQ1pgWOQRinUzPcvmeiZ4D3z4JaWm+gdbBpXnUL5R376txDsrMRp0Pdh
  t1qqPKUy7yRiXhznQsrilCIVosKRaaILy77yd5O8pyeyaO6Yn2d94u3Muahd
  zmfSR8Jzufj+S0m0RTB.QIRwJwh0Fs56dUjbdcViVwzq38jlJqM8ddFy2xNR
  iqdE.NYduOOXrTnJTPKN914OTo2NvRFLTJgSAYm.qEJa8GrZf7.fiTM9U5gn
  rpVALQxNzTVQxbVyKCZPQM8clbIcvuAq5TqvDkljyTeYsdagQO6ZDB+xrfeW
  0ZkPGbmZaCzRphHDNIxAUWNb7kKGOT20uIZJW8DxYW8ZER6Sk14MWZxRx7uu
  BJVuWqdsZqBkT0qltzRVCzNZSI8K7xvfIW9kGC35UZ0rtHZbpZa3MoWuqfWC
  7ljbcF5FkT9SP5Lo2bM8yP0WdtQX5ke4R4hC4z.6kY+zSdRtO0RW+F148xw4
  en8Mqxirpjtm1Osmnc0t7J3R3Z3Cb9SkXjDCBMOvsZ1xGnn2kqa+oIOHkCMR
  vOc2xY2MctZ8PEuar+6TRDTo+i+2kgyL9CUMntToyWjevFpcqlQR4axwGnQR
  +zU5jDVLKGiTw+HlBW68nOqch.XhmpMA0s0fn1.Lv6kCu7xf6VLe.5epVhKb
  QjgVQNgHtCqTgWsj6sAR0T+hgaKoFo+VxNz60qcYTUUmsSFNx3kNkP4b95+K
  k+EWQSJ+1WMd30wIl9EJmLLQhTRpPsqHi84mpoMsmhkUqDYrVxIauXt0UqHs
  bhtiAJdFYDPdVnZdAEz3In8iVZnRuC0z5qjZjC3xnfOqeEYosU2M6JTI.zz9
  FvQMQncZZ9K+IW.9RooFZYRyU70FGR.ZtoeeZBm4wK5Xr6EFOYg.IBrZjCOR
  rkxni0.gZIQCIdZGvnbipxfNkGDeHZtTCJPkZ0VpYj5pb1CPKM8Nib6AnyLK
  vOOg91xA4mlI67IvkWKQwH.IoaMRRGoPFkaykZlIe+WONoWXiWo3DzChZuGn
  0LbtYAEYuK1EkJAtpscPaY8JVnXGQodqlNmVa4nNl9hrNv9Qfzxk2MRaHtlS
  HpaqYFTe7.Sy81fauKUSp1ctoyFJkXOeg72WCsuwusftvq0X.sXB6jAhLsDs
  e40mJ0x8Mvi7l29z2dJ142.sV+6HpVdqV.K7+pw+TKWF4g9LhhN16yJEheXx
  dAt85EIesjLu1580E8mq8Zoq+Zaf27ZuVVAfL1Iw2s9uvzOTDf+8J+u091fP
  q2fNE8Vzuh9EiPLa+1a4692mJW0RtR1x6N9gn+PYzqzBEEW3woLu5Dz4+5a+
  ke8sJ6xV8HqrzI88AsDFbgZaINFi4DvfNsjXy8juRioHGqzw4gZtnyMVIcBX
  DhRPuTDbzVbAZILM0FMYZMs1fimb76N5mCjZoM3cGkpiDc6eT9.+lop+HE0J
  En9HcrZIelPk45m+OkeQ3aJ0nP1EONsa2kS6I0m.5zmE2.YbRu1KqpN9+IX1
  z.o1Lf1DZMHT6O6XvicPszC4.CHsIyFjC5IvmLWJD+3zuv+Sft0uSYeKnG3z
  qt5A4hJu2QgHqLT.TCVAr2jzIZHmUd6LZ4++b2otyP7Fut7KQOEcF5UkRfWK
  z2pkJOVJU9OLzfn2DsvsdQzaj5eEnVyLTswxJe0DYOi54+1uMk4a54D0m7CJ
  uCbbp6ASXqYiVFq2iagz5Kcq9OLS5Y0oyLiF6FxiAGjrxgnucpxcnlu8HkYB
  GGYEbLq2oZ+1g1vHFdXih92Humbbd8JpJfFWSAZdaR0VNNiNdFQII+NQ2x7k
  9SyK5uidp1Ji+JrI5WrLbrVgPkOr06GKZzT010IoUuNXQjeJQOSuEewpTpnj
  0pInLUwr2OKmq04ZQzWw32Fs0bQaBnQKHkhQZC3Vp2dxGnXI0fh9k9.zwQ9V
  8gmfBLaXopyAr3Q50D8hd94Z2O8im+5e5z2pEXMB9Ju6nKjr7e7en19XvadA
  v9Yp7UGJxSqGqeQQ.n9036+3nqAPcwJ6SM3ogiJREqUej4UZt9OS1pdxVUYc
  fJ+Iq2BoH+CLapZ2lx3p8UMoZcWoTG0KBk7Moah3FKc2P8cTDxoLNy48Ydhq
  y9D3rOwEYeBRxmH4aGja9L0mbrTZrjN+BfoY0iqHaT6ODJDzKMD8eY9ZR6Ez
  9MTwOhB+1u8gIw+XYxAKzOD7NBOI2WIH18zYJDWKpFBMCfpdVvUR6ct4utps
  +STVI9YMl60v2Q6rRUvIF6j6xm+yiBvQR.HaxQSi1NuUeM4K5s+7YuAI+eOE
  8lyd4u7Bo5Hm9l2hd8u9pSP+5qTg9you5sn2dN5me5+5TzSewKT8n2fNFJba
  BmGF6+zjspRXQZ2g8+Kh7vTuWy+9i79v7myDNqOIk76T3DFgy2x4mjM6el2G
  m6GJIjB1WiQmVYDBK4DqnW1amhj1FgSXP1jJ3AjZ0jrIkps8H0tOcqtdNNe5
  JCrOA16Zkz6He1oB+l4SSF3MvtpLu4DBrs3cB3rQX7WGaI.1lXu4NH.VMy59
  D2n.tcp1QypVaTnZIH8NI21.oZi3pz5RGGpk3JMUVhmmHaq+t1KzGqBkjGou
  j37vGteAZF.z50OT1hvdv7dDtVQXNrx+sCmrTpkeP9Tzoz37GVN4xaN+Jo49
  yONqVbmjUosSPYURK6mPy9kXY+.96yeojr8E7IHgr0Ix+ODl6ZQvo6+I91Ja
  tdgJjiPrbadMoflNH5QAxgii+leK5UyTu5GV9Bx4RoHm7m+wv6RSvn6ylNd9
  i4TK4tggk2dXXkd7XbPF2hQis5EDMYB5FXTN.Szge.rcXImtiL1vIGyKbxwf
  hTlwzgVHbMClB0d37ej7YVyjIMQF8gx4Sb5m7hheRR5mbqMuQxSpsXWqtC5A
  RMc1CB2RRgPNTjuUEVgUJx43D6QD0vE8re9zm8OW4IDU74jL77TtdLEC4nDz
  1QwA7zkyG+kST2R4ECfXFNAIp4.Iod3ByFLYNiBpP3e5xqMGiD8Fv+mktFze
  Je0m9pmCgRELQ7mMjWzQneD8qnWgdF5snyPmK+q2jm6F202swOi43gL87xZe
  NDiDyU9jJZWthcs2IlHyLZmufH6bNDbFYNwDZuTMJ3xPUDnEEv6CSEpkQwIp
  dyDg87aSNsSRXDAhm94P3DQoifUcCAQXQ7w5AZpiy5OyGkcGgeH56SEAR.xD
  Ee2OS2npaMBc9xXeMkzQ1o8duoUNI2Pb0bvfFf9uGpiytXu5uValpG8OJ70p
  EgGGYgZnefDaf3i5wkz9q1rxxZ8WahC5RZowSN93T6BOH4cg4LxEG1rvNzqC
  FwOMc4XUjvahhZ8hTfaGGd4BkXQ0oib9hn8XV98FXN0Px+TMNMgOti5qlMB.
  9aQwG9vQwduYT30gKFN9+dlDzSs2Pu37+6Uip7BGfj2a8Xu.tqRpg16qFoZE
  P1nEtHepXJ2YwDsfrSE45esDReCsubD+o3vSHAMeBh9zNo+6U57q0KY04wfp
  NlZoOQFqBd3nlwLMqdWR4EO4IZCcUMzSUtfVyxKmg01RDWndfswwz8VIrc8n
  vH0tJX7O85QKwpalXi5S0uLKyqOkUFGrpdrSzaeuJLBL6me33uj3Mkbq5UT6
  oZRnMeapCYUxnK+uEGl9QGSpTqc8le97W+VzKO8Mu4o+zoIzoSIMJ4K9IJsp
  UvIbHBoOR+thNtFpXXHopOlwdDLjRsHUSuxw6YzUKWgUm8pe7bHL4oQazXxS
  qxEAlPfY.5o5P9WOI+RsZmojDjyKHpGV5Coktr5P0rwVUJkIOIVYTi6Oi1LI
  f37BfbPc1yRMC81yOWx4+peZyyQeubJB8+4+CJ8mkZqVyf1kIDo7ENfiCBbv
  RgSJZJpTz5PzFEnmOrC6jp4bfBt.LSzwsgWU5sZtnGyrs1CPOe4spsyybxOV
  qo0yRpCrxusZH797ZUE8nlF8X0SeBJ4HVMwu5rqoO8eqTCQ9zQQwdzhyRoeI
  P2KhO1GINTT4PVUHUPFI.Um35+5IqQcoiQzniqB56cziI096oB0pvaCWTN4G
  NETV3Jg+7Y+zOmddIuPqK8KJNFwpnrn3.ePq91cyB.kOeL55oPT+oBmQk5c5
  IUkR4qQ7jGyODRqqN0Uot4CRbV0dRFYCpMCAv6aiNVXQmJLfMY0o.KTuKsxu
  8.zYvgS7S5nBbRx.1yDodJKSxb95NI4YCV2WzDuRUhBTNMPGfZlieWxc6TES
  DOd0H9a+1zqX9ORZ2XQhYJ7DvBh+iaw0DImUbbpW8pGLVanGjPHrDtzn0Cxn
  wzC9dB7dePjFRlXA3cGslVNu6Ho0XFcnVytEvXsL15rZm8yXrSL6Qn4Llc4P
  8YCSGpfqaSj9L9jkyR4gE7CTS+5iDCz.x44UGclnif33jG2rDxZhDxDwJ8se
  qwr6OcSnTotikRKVO5JVI0HM+4+URwgODEy2kxx9Tek2ullTv2H0Co6SF1LE
  mcFgBOI4aEXcVEtwq5.4DjsH3TR8fHElU26AZ+.j2AFDZJaTiOiZRQ9OKO82
  ijLVnDqbkU8bUzY8CQRkztdynmszHr6FGrHHIiC3VAC0dRn4OSPwtJnchNbC
  OTGO35cYHJZ2N84Z8BTgLaTftFF6oiAn2pS9XyQmO46N+pqRPEZBtnns2NdW
  s+ibvzLQV2p0DzCin8J1huY7Dh7K9mFl53QYhQd1fDJQjPo5vqGg5+0mjNBj
  z5Oe9pv8MS7IAgSlQxoIPajbd2LbNnh6PiajxFaZGqOtk4Fpdoe8QQvTj+uf
  03SPLo3T+qIOuJfJ+Hce9Zos0PP5m2ILR1VoNnKv4nVm5NSFCSoDxqMP7GN8
  mN6UfkXOZ5UORKKJ0AgQ8I+sTB3yLHyD7X48l9knyGPzAlor1KqWXdH5Qvr6
  pvQK2up4ck4auoddxWTwMZFSRKqQy5wnhaznPaqrVKc3ukayjwn7xZsbnGyq
  IKa0cqVYOmdYjP1xZrrVClSyjzz5R5UqYeSAyowNEL5.zDc3M1zLbR2trwWg
  5IquFTs7+lZM0yTRyroIhhlDhWXCbq5qdNp.IGwMcR+coEdc85BuRtjSrjOs
  fuwARspikFYhzUHAZD6wiBDATeRYh0mI978ActeTMbR43VcaOExAR2DGrg5u
  1amBu+Lwh85VGC50JQ1zK2kX6EjRSkulTs5iQOM04jS8.5yFurunQcyoKzbB
  zLGNn6lNagYs0z8QyASV9Q+16OQu+huxnoyZwj8eFER1O6Em9zW+F3EoXjhB
  1x711vWgf8ILgBBYVP8OPQqnt16D5WgxN1yO8YqTbHB1VGbRnrepHNU9MRbZ
  5e14u5GO6mFXjlJU27wINEvm.G7RoFGqi.RiklHW8SqW5po.ipBlAQpySbdA
  OZD.kWXipH8zgxgyp.Rd0QCN53XF99bT9JKrFaXWbC.J878H+nt5eFEW2INy
  r4EppIOrqY1yP88ybNsi5lr2mTOoDmeh02Ci056oyuARh4GiJPuqDJUkeSEO
  92PCkd420ZlTCxMzTYGfwTroHKA8cSr2UJc4+we8UOSkb5dSRR4ja0q9LDYB
  ABCCqNJHf+FhUCc3D.+5Z3WWXnmVwcFyvVkXrnnPE4X06WsQ9OrnMeOAObvj
  rCtjIWwTYBblKuyjcJIvuXd5Tstf5xJIAUR19DTYtUs76u4d+hxd7Qo3BPTS
  8kbpKHsmycz0oDpiq5WdtYS8nVlTqILcRslg0YI9bZFKxs4XLtcRP+mOwZXk
  fG3hIBNoJXKkjH66KF3w7888je.d6gWtFd0YFalt8R0H1.tNsT0O.6DktCoZ
  cjjqaJUa7aQ+as8Glb8y6l.e9ie2D0CieL33Y0eSf7zCbA8wnnrhn5RlTAVs
  mZOd0Vk8spr2j5xbK1Btsaxc2E6qlxcF35QXpbZNA6N.qqdBNNETUJ7ZkL79
  hBpnG3cshdfq+BjfI03apPBEjH34tsDt4r6xfqSLR3rEXDuUvHc8ho.bhtqz
  WzsuhwXUkiIlXyw0HetLfjUu.4lJsI0WYMAVCR3rp73n9YcVVShWbRDsn+1u
  zO0uKTVSrsjlzQKmIyPQtCX2KfGagDqRINzEUGBT86D3xXw7ZoBchxiPRAVp
  d3ljWUmUypKUoKxMJTB6S.gRfQVtkVJTnzutJEJzDEBEATGTXdN80Ak95fRe
  cPojB3QJIgYUtU64t5sBnf2nOlVS3HzKR4LobDNZpcrPURgs1hIqIdD2dEIk
  RwXin+8LFC8hMgwXhaRPltQP1oS.xlD0LRt9k7W6crdivroD+.tFx2aSnLta
  fxpbX2AC15.EVO2Mgs9cBnclbbFn0MsP.lTE.lTA.9DqAYrqFjEjMAxdcJoD
  kBycF5XLEDECXLn1aYXrn8vXMZUHBtOX2AyAH7neVDHQRgRQ9vd0V4bvgOZe
  csQRIGxVARt2y.Ia40rFf7ZC.BFFVInpF1kWOJ3EOL2RLv+9GFX7Er0X.krM
  XPtavcVXQ2qT2OevY9zkytLHQUt2eshb+HcoQL1QG+1JUHy7f2DNZTvjrfUT
  H0p+bqlI21trmkcY+NSOVXYOVQOzQ5xJqeO35yNV1mIcGRChsTyhtSW10VRC
  7AXet6HmiXqXCuNSWF25bf2eCvqkRkAVl2V7l5FVrCuQaIU3hfaMZMbjJk.t
  X5zE27nyLI6hG8KZqZQGch5GKmegZWUtEyv3i11c68jLNAnzsOG1oJiCCKZq
  pb2uauaRDujc2svGqCr4tl3YeHBxR6aA4y5aRobt7hgajBvl87mHzdH1mT59
  TVi6e6+qDzmMEUDBPZn83NkU1kAHBXeaKcmtIsxNcOd5vQPfvzhQuzFwIB1O
  Y3KIJUpQMGNnli.+5wMg5j21pjSkBQRRHMBIzwNAubDxssfHcFeq6.RZrwNH
  xqcfHOJihKWpTy.EDNUSvnCH+MgFR8nqww7dY7h872hwKqVGuBBaeLhMQWuU
  iXQkVt81vYCGbkRmlntX30SlJ6BiCu7iYzsX8PJawvKVwXMd7GdwoO+C5C.y
  GRKY09PmWfoC3999tBIatd0TWW+AX4mvwRDPRx6KmKDzXDXgp7JOJXVpPhbc
  nzWJYxyi6xHRDzSuFDV9pvXFwkdBxyY.C6hc4wsq53Id8LU8runfsL4mqKqA
  vaxYfuT2aU+2Yfv2A6q+K4ubb8T+EU8Q3bkXQ462oveAxmBUZhyafqiiClDM
  sw8ck.AwyUNswjJIopVhNhbm1hZ4QyF9oOHuwGh5PNkDv89z3WXCLelTK1TR
  RzsWZ+3ByQYs31LSstk1zTV8Wj2LTOzJzZMSrkS5itapp.aZXa3Xc.W5CgJG
  C1zP+DmTq0MHeMxL6GBTaGB7seHHLCAPUZQyLDDXaGArsXDPLp+CabqIPDRc
  EELW1kRiupIlfR6ApxFdzsdBRsL7pAT5qhFdDdyN7rl9ir8COXU1ngWpqhFd
  X2lb346a6nCuEiNHTrwNdIGcoth5XlJ8iupIl7brdxyYqm7bIvH.TbfzPTeL
  KF.X5FlepyNDtq0gb5XcHreWqCI5ZcHRWiFxMkFPEs41jMHmnF6QbKPnLrhM
  Y+wNlLdq0e7qEwhYjqCAhrKSufqGYsKTx8MKEq8BKG2Hx78rQmcri+FTnpF6
  P1vtRnts1ruGt4l8wIm8w6gYeaTXo8PZa.Zu1iu2ka0Lue60grQvHm0d8mTD
  8EP9P2NiUntvtvRLG6Ve8QjI8UXWHF34vNTourILklzXb9BemDL6otnc3749
  M2XSjb3H1CiMWKFas2BHbd2p6Xi1L7Vt+rIoH71SpOu4VuWPAlAu0unk3L5X
  lux7Z.rVsbfFDolcUhj6ksChybaxQnIXCvhburkFghlbD5mdH4uWFg7FbDZT
  6OZHk4x1ZDRaxQHK8HjsWFgDKVkgzdB9rQPre6Y5kU8GJua0eHaquz87wPz0
  .VZKLwdPxqXvAX1kPhupAnFo1n8sKe6FcXJX4DLPHXy5br82Hn8bgI0lkYI7
  tU+AKbZuNjUFgHpF+DWqCaxqZBJNZMXFUlA.wACA8aj6EXo88.bowcCTJqQc
  2.kZi6TZOJFqBjf1yNQpsNaZql+gHSk3YRXEvuReEmlHb.ReE1CRmHFxH8kM
  AkgMBSHs2TAwFgIr1S3FwpHOhvZO86H1sfjNhA1hfgBy4ZGfxgbm.lBponur
  An6HVtLFt01cHK6Q58qZGvUGedLtt5Ri80XGG+U1a2HvtMRdaw8iUGWHjMty
  Lrsz9PFgoSQAvxqotrIPUhMNZTJBeaGERKWSNJjWRk+yk0zCkMxE35ukCEpq
  H4PQcoWiOTrXj3QZwXOvpkRb3sbOBa0RIsjPXmlvXBrIg86CghnwZB+FJTDI
  VErdvHskTYvwpHevwqEo7vctdjSWqG0ohfBrUQXaahOXeuNWOxsy0irZKGcw
  sXOxpMJTzhXjmUAr.tM6QVs4k9s3rlmUFG46zh8Hq1bPO+VrGY0l441hQeqc
  AXnWaRY20VkE2sBTQrqW2x2nV1gZQQQtVIJxqMgHQWin1t3K0Szh8HZmCirS
  0ns7Dwg4fulnFO4IulLf533v7i9vl37U414jq51sjqJ7rd1dqbQGljZ1Vcsu
  b1lvZxYaQmSfif045Q1ErkNs2oyR3zwN+IV1iHrszcals7fQDwN6M5xlfafa
  0lXtwsnKynfKfRcFChubX+ZzW0HiAqNP.7VTfocAtG2qq0i154YnhRvbDImm
  cZHZUqBtI91FqIXHcTiIPFR.RoBYtBxMDwQZBCuGGf31THG02lMmqE6OLqWX
  ps5QNMwltnKuwlrRT6ryKXaDPvZQOBoCbFuMzgD7VtCsosaWrs6SsvjS0fPF
  AaNlcYuDpW.XgetW5Ztju5xFIOT3lJA6XQ1mtElPb2jYRNsbGhsIUBc2RJDI
  eOPR.AnuOO2KcMwFoo3Jznz.zMFRWa+QRPOHfBuHgk+kQEoReyPj1fCQgM4c
  g1LyTrwEg2VEevL+jnLESy6RFlDE6kQW0XX9l2FoVL8jXuKO1tfoflxkGott
  I.Vrc.a6c3zy9tpGEl78YCDx+QZO8kb7sFYaIZVG6CBSdK0iXV2iXsTOpASa
  HtISaHtseZCwpSFPUOEwPdnLSYaV0SxTtlyTplWuLMWbIZNa4YV9l+yu4+en
  kwcvF
  -----------end_max5_patcher-----------
  </code></pre>

*/

