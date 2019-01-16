//
//
// Flying Squirrel #3 flight computer software
//
// Author Don Gibson KJ6FO

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
// ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
// CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//

#define FLIGHTHARDWARE 1  // True for FS3 flight hardware false for proto board/development hardware
#define DEBUGMESSAGES 1  // Enable debug messages. Set to False (0) to save program memory space
#define DEBUGSCHEDULE 0  // Set to 1 to enable a faster telemetry schedule to assist debugging.
#define CALIBRATIONMODE 0 //Set to 1 to enable calibrartion routine.

#include <OneWire.h>
#include <uBloxGPS.h> // https://github.com/SquirrelEng/uBloxGPS
#include <si5351.h>  // https://github.com/etherkit/Si5351Arduino
#include <SoftwareSerial.h>
#include <KJ6FOWSPR.h> https://github.com/SquirrelEng/KJ6FOWSPR
#include <string.h>

void PositionUpdateCallback(uBloxGPS *gps); // Callback function for uBlox gps code


#if FLIGHTHARDWARE
    // Define adjustments and device addresses on the actual flight hardware. 
	#define CALIBRATIONADJUST  -11300L // Measured from calibration program.  FS3 Flight hardware

// Adresses of DS18B20 temp probes
byte InsideTempAddr[8] =  { 0x28,0xFF,0xC6,0xBF,0x80,0x14,0x02,0xF4 };
byte OutsideTempAddr[8] = { 0x28,0xFF,0x9D,0x94,0x3E,0x04,0x00,0xED };
#else
// Define adjustments and device addresses on the development/prototype hardware. 
#define CALIBRATIONADJUST  -18627L // Measured from calibration program.  Proto board 1

// Adresses of DS18B20 temp probes
byte InsideTempAddr[8] =  { 0x28,0xFF,0x3C,0x1A,0x80,0x14,0x02,0xC7 };
byte OutsideTempAddr[8] = { 0x28,0xE1,0xE4,0x99,0x04,0x00,0x00,0x29 };
#endif


// Mode defines

//
// WSPR
//
#define WSPR_DEFAULT_FREQ       14095600UL + 1400UL  //Hz  Base(Dial) Freq plus audio offset to band pass bottom edge.
#define WSPR_DEFAULT_FREQ_ADJ         75L // 75Hz up from bottom edge of 200hz passband 
#define WSPR_TONE_SPACING       14648+5   // ~1.4648 Hz + 5 to handle interger rounding.
#define WSPR_TONE_DIVISOR       100       // Divisor to bring the number to 1.46-ish
#define WSPR_DELAY              682667UL     // Delay nS value for WSPR .682667s  minus time it takes to change frequency (Measured varies by CPU board, but 5 seems to work weel for most)
#define WSPR_POWER_DBM				13    // 13 dbm = 20 Milliwatts          
#define WSPRType1 1  // Should be  enums, but having compiler issues
#define WSPRType2 2
#define WSPRType3 3


//
// FSQ
//
#define FSQ_DEFAULT_FREQ        (14097000UL + 1350UL)    // HZ Base freq is 1350 Hz higher than dial freq in USB 

#define FSQ_TONE_SPACING        879          // ~8.79 Hz
#define FSQ_TONE_DIVISOR        1            // Divisor of 1 will leave TONE_SPACING unchanged
#define FSQ_2_DELAY             500000UL         // nS Delay value for 2 baud FSQ
#define FSQ_3_DELAY             333000UL         // nS Delay value for 3 baud FSQ
#define FSQ_4_5_DELAY           222000UL         //nS Delay value for 4.5 baud FSQ
#define FSQ_6_DELAY             167000UL         // nS Delay value for 6 baud FSQ


//
// Hardware defines
//
#define XMITLED_PIN             13
#define RADIOPOWER_PIN			11
#define GPSPOWER_PIN			10
#define DS18B20PIN				12		
#define OFF 0
#define ON 1



//
// Global Class instantiations
//
Si5351 si5351;
KJ6FOWSPR jtencode;
uBloxGPS uBlox;  // The uBloxGPS object.

OneWire  ds(DS18B20PIN);  // on pin A3 (a 4.7K resistor is necessary)

//
// Global variables
//

char MyCallsign[] = "MYCALL/B";  // Your Callsign. /B is just to indicate it is a balloon. Not a standard.
char MyWSPRCallsign[] = "MYCALL";  // FS3 only uses standard callsign. (i.e. not compound)

uint8_t RadioPower_dbm = WSPR_POWER_DBM; 
#define TXBUFFERSIZE 162  // Big enough for WSPR and our FSQ Telem. Adjust if FSQ Telem gets longer
uint8_t tx_buffer[TXBUFFERSIZE];


//
// Global GPS Info
//
// Current values of time and position etc.

long LastLat = -1.0;  // Last reported Lat
long LastLon = -1.0;  // Last reported Long
long CurrLat = 0.0;  // Current Lat
long CurrLon = 0.0;  // Current Long
long CurrAlt = 0;    // Current Altitude
char CurrentGridSquare[7];  // 6 digit grid square
float CurrentInsideTemp;  //Temp from controller board DS18B20
float CurrentOutsideTemp; // Temp from external DS18B20 probe
volatile int CurrFix = 0; //  Fix number (i.e. # of fixes from GPS)
volatile int WorkingFixNum = 0; // Used by GetFix to count fixes obtained.

volatile int CurrDays = 0; //  # of Days
volatile int CurrHours = 0; //  # of Hours
volatile int CurrMinutes = 0;
volatile int CurrSeconds = 0;
int LastSecond = -1;   // used by time debug messages in loop() 

//
//Battery Voltage  & Power management
//
// The Voltage threshold value we have to have to stay running.
//#define CUTOFFVOLTAGE 2.8  // About the minimum to restart GPS and allow power until a solar charge
#define CUTOFFVOLTAGE 0  // FS#3 has no solar, so let run till done.

// The treshold voltage value we need to get started up from a boot.
//#define TURNONVOLTAGE 2.9
#define TURNONVOLTAGE 0 // FS#3 has no solar, so let run till done.

float CurrentVolts = 0; // Measured battery voltage
bool bRadioIsOn = false;

// Use software serial to send commands to GPS
SoftwareSerial Serial2(8, 9); // RX, TX



///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////// CODE ////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

//
//  Setup() Lets get going!!!
//
void setup()
{
	
	// Set up serial port
	Serial.begin(9600); // Baudrate determined by GPS module
	
#if DEBUGMESSAGES
	Serial.print(F("FM=")); Serial.println(freeMemory()); // Check Free memory
#endif

	// Enable internal reference so we can measure battery voltage.
	analogReference(INTERNAL);// Use internal 1.1v reference for analog pins.

	// Use the Arduino's on-board LED as a keying indicator.
	pinMode(XMITLED_PIN, OUTPUT);
	XmitIndicator(OFF);

	// Init Power pins
	pinMode(RADIOPOWER_PIN, OUTPUT);
	RadioPower(OFF);
	pinMode(GPSPOWER_PIN, OUTPUT);
	GPSPower(OFF);
	uBlox.SetPositionUpdateCallbackFunction(&PositionUpdateCallback);


#if CALIBRATIONMODE
	// Run calibration tones forever. 
	CalibrationTone();  // Measure frquency while this test runs and adjust CALIBRATIONADJUST define until it is right.
#endif

	// FS does not use a solar cell, so this code is inert. 
	// We may be restarting from a cold dead battery that is begining to charge from the solar cells at first light.
	// Wait until we have built up enough voltage for a fix to succeed and get stuff running.  If the voltage is too low
	// the GPS will kill the battery before getting a fix and possibly cause reboot again in an near endless cycle.
	GetCurrentVolts(); // First battery Reading is always high. Read it and move on
	delay(1000);
	GetCurrentVolts(); // This will be a more accurate reading.
	while (CurrentVolts < TURNONVOLTAGE)  // Just loop here until we have enough power to get going.
	{
		GPSPower(OFF); // Make sure GPS Stays off, it may have tried to start and the MCU crashed leaving it on.
		GetCurrentVolts();
		delay(10000); // 10 sec
	}

	// set the data rate for the SoftwareSerial port. Rate depends on GPS module baud rate.
	Serial2.begin(9600);

	// Get our initial position and get the time clock up and running.
	GetFix(30);
	
}


//
// The Loop()
//
void loop()
{

	// Only process once each second
	// Current seconds is updated in GPS call back routine.
	if (CurrSeconds != LastSecond) // Compare current value to last value processed
	{
		LastSecond = CurrSeconds; // Save the new current value.

		Serial.print(CurrMinutes);Serial.print(F(":"));Serial.println(CurrSeconds); // Display time on console

		if (CurrSeconds == 59) // at 59 seconds...
		{
			GetCurrentVolts(); // Get a voltage reading
		}

		if (CurrentVolts > CUTOFFVOLTAGE) // Do we have enough power to do stuff?
		{
			if (CurrSeconds == 0)  // Top of the minute
			{

				// Process the schedule
				int SchedMinute = CurrMinutes % 30; // # of minutes into schedule (30 Min repeating schedule)


#if DEBUGSCHEDULE
				//DEBUG SCHEDULE
				//  Cycles through modes one per time slot, faster to debug 
				//

				SchedMinute = CurrMinutes % 4; // Set to 4 for full debug schedule. 2= just GPS & FSQ - Test of GPS syn times
				Serial.print("SchedMinute="); Serial.println(SchedMinute);
				switch (SchedMinute)
				{
				case 0:
					GetFix(10);
					break;

				case 1:
					XmitFSQ(FSQ_4_5_DELAY, true);
					break;

				case 2:
					XmitWSPR();
					break;

				
				}

				/*if (CurrMinutes % 2 == 0)
				{
					if (MsgType == 1)
					{
						MsgType++;
						XmitWSPR(WSPRType2);
					}
					else if(MsgType==2)
					{
						MsgType++;
						XmitWSPR(WSPRType3);
					}
					else
					{
						GetFix(10);
					}
				}
				else
				{
					MsgType = 1;
					XmitFSQ(FSQ_3_DELAY,true);
				}*/
#else
				switch (SchedMinute)
				{
					// FSQ 4.5
					case 1:
					case 17:
					{
						XmitFSQ(FSQ_4_5_DELAY,true);
					}
					break;


					// WSPR Type 1
					case 2:
					case 18:
					{
						XmitWSPR();
					}
					break;

					// Long GPS fix time
					case 10: // One per schedule period, run the GPS longer so it has time to update Ephemeris/Almanac Data from sats. This will keep the GPS running faster
					{
						GetFix(45);
					}
					break;

					// All the rest
					default:
					{
						if (CurrMinutes % 2 == 0)  // Even minutes
						{
							GetFix(20); // Quick Fix.
							
						}
						else // odd minutes
						{
							XmitFSQ(FSQ_6_DELAY,false);
						}
					}
					break;
				}
#endif
			}
		}
	}
}


// Xmit a carrier at the bottom of the WSPR subband. Use to adjust si5351A board with calibration adjust.
void CalibrationTone()
{
	for(;;)
	{
#if DEBUGMESSAGES
	Serial.println(F("Calibration Tone"));
#endif
	CurrentInsideTemp = GetCurrentTemp(InsideTempAddr);  // Required for Frequency temp compensation adjustments
	bool bLeaveRadioOn = false;
	StartRadio(); // Init Radio.
	delay(1000);
	XmitIndicator(ON);  // Turn on the ON AIR light.
	si5351.output_enable(SI5351_CLK0, 1);  // Radio xmit enabled. 

	unsigned long  XmitFreq = WSPR_DEFAULT_FREQ;
	uint64_t AdjustedXmitFreq = (XmitFreq * SI5351_FREQ_MULT);  //Convert to 100th Hz
	AdjustedXmitFreq -= CalcFreqDriftDelta(CurrentInsideTemp); //adjust for temp

	
	si5351.set_freq(AdjustedXmitFreq, SI5351_CLK0);
	delay(30000);  // 30 Seconds.
	

	// Turn off the radio outout
	si5351.output_enable(SI5351_CLK0, 0);
	XmitIndicator(OFF);  // Turn off xmit light
	RadioPower(OFF);

	delay(5000);  // 5 sec pause

	}
}




//
//  Encode and send FSQ telemetry.
//
void XmitFSQ(unsigned long BaudRate,bool bLeaveRadioOn)
{
	char Message[90]; 

#if DEBUGMESSAGES
	Serial.print(F("FM=")); Serial.println(freeMemory());
#endif

	StartRadio();  // Init Radio.
	delay(1000);

	// Clear out the transmit buffer
	memset(tx_buffer, 0, TXBUFFERSIZE);

	// KJ6FO/B FS#2 FFFFF HHH:MM 999999.9m -99.9999,-999.9999 -99.99c -99.99c 9.99v  // Message format

	// Update Temp 
	CurrentInsideTemp = GetCurrentTemp(InsideTempAddr);  //Inside temp, required for Frequency temp compensation adjustments Telemetry
	CurrentOutsideTemp = GetCurrentTemp(OutsideTempAddr); // Telemetry only

	// Compose Telemetry message
	char fbuff[12];  // Working buffer
	strcpy(Message, "[fs3.txt]hab "); // Direct FSQ to save data in fs3.txt and indicate this is a High Altitude Balloon.

	strcat(Message,itoa(CurrFix, fbuff,10));  // GPS Fix Number
	strcat(Message," ");
	strcat(Message, itoa(CurrDays, fbuff, 10));  // Days. This value wil be the UTC Day of the month
	strcat(Message, ":");
	if (CurrHours < 10)  // UTC Hours
	{
		strcat(Message, "0"); //leading zero
	}
	strcat(Message, itoa(CurrHours, fbuff, 10));
	strcat(Message, ":");
	if (CurrMinutes < 10)  // UTC Minutes
	{
		strcat(Message, "0"); //leading zero
	}
	strcat(Message, itoa(CurrMinutes, fbuff, 10));
	strcat(Message, " ");
	strcat(Message, itoa(CurrAlt, fbuff, 10));
	strcat(Message, "m ");

	// Format Lat & Lon position
	double d = CurrLat / 10000000.0;
	dtostrf(d, 1, 6, fbuff);
	strcat(Message, fbuff);
	strcat(Message, ",");

	d = CurrLon / 10000000.0;
	dtostrf(d, 1, 6, fbuff);
	strcat(Message, fbuff);
	strcat(Message, " ");

	// Format Temperatures
	dtostrf(CurrentInsideTemp, 1, 1, fbuff);
	strcat(Message, fbuff);
	strcat(Message, "c ");

	dtostrf(CurrentOutsideTemp, 1, 1, fbuff);
	strcat(Message, fbuff);
	strcat(Message, "c ");

	// Volts
	dtostrf(CurrentVolts, 1, 2, fbuff);
	strcat(Message, fbuff);

	strcat(Message, "v     \b");

#if DEBUGMESSAGES
	Serial.print(F("MSG=")); Serial.print(Message);
	Serial.print(F("MSGLen=")); Serial.print(strlen(Message));

	Serial.print(F("FM=")); Serial.println(freeMemory());
#endif
	uint8_t symbol_count = jtencode.fsq_dir_encode(MyCallsign, "allcall", '#', Message, tx_buffer);
#if DEBUGMESSAGES
	Serial.print(F("Symbs=")); Serial.println(symbol_count);
#endif

	// Send FSQ Message
	SendTelemetry(FSQ_DEFAULT_FREQ,tx_buffer, symbol_count,FSQ_TONE_SPACING, FSQ_TONE_DIVISOR, BaudRate, true);

	// To keep the radio chip "Warmed up and frequency stable, leave the radio on
	// if we are following up with a WSPR transmission next. 
	// Otherwise, turn the radio off to save power.
	if (!bLeaveRadioOn)  
	{					 
		RadioPower(OFF);  //Power down Radio
	}
	
}

//
//  Send WSPR reports.
//
void XmitWSPR()
{
	StartRadio(); // Init Radio.

	CurrentInsideTemp = GetCurrentTemp(InsideTempAddr); // Required for Frequency temp compensation adjustments

	uint8_t symbol_count = WSPR_SYMBOL_COUNT; // From the library defines

	// Clear out the transmit buffer
	memset(tx_buffer, 0, TXBUFFERSIZE);

	// Encode message 
	CurrentGridSquare[4] = 0;// Truncate to 4 digits.
	jtencode.wspr_encode(MyWSPRCallsign, CurrentGridSquare, RadioPower_dbm, tx_buffer);  // Type 1
			
	// DEBUG Remove.
	// Dump Symbols, use to verify vs wsprcode.exe tool.
	//Serial.println(F("WSPR Symb:"));
	//for (int i = 0; i < TXBUFFERSIZE; i++)
	//{
	//	Serial.println(tx_buffer[i]);
	//}

	SendTelemetry(WSPR_DEFAULT_FREQ + WSPR_DEFAULT_FREQ_ADJ ,tx_buffer, symbol_count, WSPR_TONE_SPACING, WSPR_TONE_DIVISOR,WSPR_DELAY, false);

}

//
// Xmit the encoded telemetry symbols
//
void SendTelemetry(unsigned long XmitFrequency, uint8_t *tx_buffer, int symbol_count, uint16_t tone_spacing, uint16_t tone_divisor, unsigned long tone_delay, bool bTuningGuide)
{
	unsigned long millicount = 0L; // DEBUG

	// Start Transmitting.
	XmitIndicator(ON);  // Turn on the ON AIR light.
	si5351.output_enable(SI5351_CLK0, 1);  // Radio xmit enabled. 

	uint64_t AdjustedXmitFreq = (XmitFrequency * SI5351_FREQ_MULT);  //Convert to 100th Hz
	AdjustedXmitFreq -= CalcFreqDriftDelta(CurrentInsideTemp); //adjust for temp

	// Radio oscillator drifts up in freq as ambient temp cools down. Need time to fine tune signal on Rx.
	if (bTuningGuide) // Xmit tuning guide? USed for FSQ to dial in receiever before first char. Not a standard
	{				  // part of the FSQ protocol, but very usual when balloon freq drifts with temperatures.

		// Xmit tuning guide tone for FSQ
#if DEBUGMESSAGES
		Serial.println(F("Tune Start."));
#endif
		si5351.set_freq(AdjustedXmitFreq, SI5351_CLK0);
		delay(5000);  // 5 Seconds.		
	}

	// Now transmit the channel symbols
	for (int i = 0; i < symbol_count; i++)
	{
		unsigned long starttime = micros(); // Start time
		si5351.set_freq(AdjustedXmitFreq + ((tx_buffer[i] * tone_spacing)/ tone_divisor), SI5351_CLK0);

		while ((micros() - starttime) < tone_delay) {}; // Spin until time is up
		
	}

	// Turn off the radio outout
	si5351.output_enable(SI5351_CLK0, 0);  //Stop transmitting
	XmitIndicator(OFF);  // Turn off xmit light
}


//
// Set up interrupts for timer
//
// reference http://www.instructables.com/id/Arduino-Timer-Interrupts/
//
void SyncClockInterrupts()
{
	cli();//stop interrupts

	//
	//set timer1 interrupt at 1Hz
	//
	TCCR1A = 0;// set entire TCCR1A register to 0
	TCCR1B = 0;// same for TCCR1B
	TCNT1 = 0;//initialize counter value to 0
			  // set compare match register for 1hz increments
	//OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)  16Mhz Based systems
	OCR1A = 7812;// = (8*10^6) / (1*1024) - 1 (must be <65536)   8Mhz Based systems

				  // turn on CTC mode
	TCCR1B |= (1 << WGM12);
	// Set CS10 and CS12 bits for 1024 prescaler
	TCCR1B |= (1 << CS12) | (1 << CS10);
	// enable timer compare interrupt
	TIMSK1 |= (1 << OCIE1A);

	sei();//allow interrupts
}

//
// One second (1Hz) clock tick ISR
//
ISR(TIMER1_COMPA_vect)   
{
	CurrSeconds++;
	if (CurrSeconds == 60)
	{
		CurrSeconds = 0;
		CurrMinutes++;
		if (CurrMinutes == 60)
		{
			CurrMinutes = 0;
			CurrHours++;
			if (CurrHours > 24)
			{
				CurrDays++;
			}
		}
	}
}




//
// Callback function to be called when a valid position is decoded.
//
void PositionUpdateCallback(uBloxGPS *gps)
{
//#if DEBUGMESSAGES
	Serial.print(F("F#=")); Serial.println(WorkingFixNum); 
	//Serial.print(" hAcc="); Serial.print(gps->PVT.hAcc); Serial.print(" vAcc="); Serial.println(gps->PVT.vAcc);
//#endif
	WorkingFixNum++;
}


//
// Get position altitude andtime fix from GPS
//
// nFixes should be 45 or less.
void GetFix(int nFixes)
{
#if DEBUGMESSAGES
	Serial.println(F("GetFix()"));
#endif 

	// Turn on GPS
	GPSPower(ON);
	WorkingFixNum = 0;
	delay(100);  // Make sure GPS is warmed up
	SetUpGPS();


	int TimeoutSeconds = 0;


	//Get GPS Data
	// Collect data until we get the requested # of fixes. This allows the GPS to settle on a more accurate fix
	// from a cold start.
	while (WorkingFixNum < nFixes)
	{
		while (Serial.available() > 0)
		{
			uint8_t ch = Serial.read();
			uBlox.FeedMe((char)ch);
		}

		// If the clock is ticking (i.e any time after first fix), dont let the lack of a GPS Fix
		// prevent this function from returning. It is better to repeat a fix than hang up here looking
		// for a fix that may not come. (Maybe the GPS failed for some reason). This will keep telemetry 
		// active and we will know from the repeating fix# what is going on. We can try to locate via
		// t-hunting if FS is on the ground after a hard landing.
		if (CurrSeconds == 59)  // time out at the last second, abort return. No fix update
		{
			GPSPower(OFF);
			return;
		}
	}


	SyncClockInterrupts(); // Refresh Clock interval to sync with GPS in case of drift.	

	// 4th Dimention is time.
	//uBlox.PVT.sec = 58; //DEBUG REMOVE
	CurrSeconds = uBlox.PVT.sec;	// Update seconds
	CurrMinutes = uBlox.PVT.min;	// Update minute
	CurrHours = uBlox.PVT.hour;		// Update hour
	CurrDays = uBlox.PVT.day;			// Update day

	// 3D cooridinates
	//gps.get_position(&CurrLat, &CurrLon);
	CurrLat = uBlox.PVT.lat;
	CurrLon = uBlox.PVT.lon;
	CurrAlt = uBlox.PVT.hMSL / 1e3; // Centimeters to meters
	Calc6DigitGridSquare(CurrentGridSquare, CurrLat / 1e7, CurrLon / 1e7);

#if DEBUGMESSAGES
	Serial.print(CurrLat); Serial.print(F(",")); Serial.println(CurrLon);
	Serial.print(F("A=")); Serial.println(CurrAlt);
	Serial.print(F("G=")); Serial.println(CurrentGridSquare);
#endif

	CurrFix++; // Bump Fix count

	// Turn GPS Off
	GPSPower(OFF);

//#if DEBUGMESSAGES
	Serial.print(F("FM=")); Serial.println(freeMemory());
//#endif
}


void SetUpGPS()
{
	char buff[44];

	// These should be turned off, but memory is an issue, so we can leave the NEMA message on. No really bad effects from this
	// And the code works as a result.

	// Turn off default NEMA messages
	/*static const char  Disable_NEMAGLL[] PROGMEM = { 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2A };
	memcpy_P(buff, Disable_NEMAGLL, 16);
	Serial2.write(buff, 16);

	static const char Disable_NEMAGSV[]  PROGMEM = { 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x38 };
	memcpy_P(buff, Disable_NEMAGSV, 16);
	Serial2.write(buff, 16);

	static const char Disable_NEMAGSA[]  PROGMEM = { 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x31 };
	memcpy_P(buff, Disable_NEMAGSA, 16);
	Serial2.write(buff, 16);

	static const char Disable_NEMAGGA[] PROGMEM = { 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x23 };
	memcpy_P(buff, Disable_NEMAGGA, 16);
	Serial2.write(buff, 16);

	static const char Disable_NEMAVTG[]  PROGMEM = { 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x46 };
	memcpy_P(buff, Disable_NEMAVTG, 16);
	Serial2.write(buff, 16);

	static const char Disable_NEMARMC[]  PROGMEM = { 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x3F };
	memcpy_P(buff, Disable_NEMARMC, 16);
	Serial2.write(buff, 16);*/

	// Turn on the NAV-PVT message, this is required.
	static const char Enable_UBXPVT[]  PROGMEM = { 0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1 };
	memcpy_P(buff, Enable_UBXPVT, 16);
	Serial2.write(buff, 16);

	// Turn on the NAV5 High Altitude dynamic platform model message
	
#if 1
	//The message is broken into 5 parts to keep buffer size small (Running out of ram)
	static const char Enable_UBXNAV5[]  PROGMEM = { 
	0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03,
	0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00,
	0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x52, 0xE8
	};
	memcpy_P(buff, Enable_UBXNAV5, 44);
	Serial2.write(buff, 44);
#else
	//Part 1
	static const char Enable_UBXNAV5_1[]  PROGMEM = {
	0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03
	};
	//Part 2
	static const char Enable_UBXNAV5_2[]  PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00
	};
	//Part 3
	static const char Enable_UBXNAV5_3[]  PROGMEM = {
	0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C
	};
	//Part 4
	static const char Enable_UBXNAV5_4[]  PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};
	//Part 5
	static const char Enable_UBXNAV5_5[]  PROGMEM = {
	0x00, 0x00, 0x52, 0xE8
	};

	//Load and send the 5 parts
	memcpy_P(buff, Enable_UBXNAV5_1, 10);
	Serial2.write(buff, 10);
	memcpy_P(buff, Enable_UBXNAV5_2, 10);
	Serial2.write(buff, 10);
	memcpy_P(buff, Enable_UBXNAV5_3, 10);
	Serial2.write(buff, 10);
	memcpy_P(buff, Enable_UBXNAV5_4, 10);
	Serial2.write(buff, 10);
	memcpy_P(buff, Enable_UBXNAV5_5, 4);
	Serial2.write(buff, 4);
#endif

}

// GridSquare6Digits should be 7 bytes long to allow for null terminator.
void Calc6DigitGridSquare(char *GridSquare6Digits, double lat, double lon)
{
	int o1, o2, o3;
	int a1, a2, a3;
	double remainder;

	// DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG
	//lat += 2.0; // Delibrate error to test moving grid squares
	//lon += 1.0;
	// DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG


	// longitude
	remainder = lon + 180.0;
	o1 = (int)(remainder / 20.0);
	remainder = remainder - (double)o1 * 20.0;
	o2 = (int)(remainder / 2.0);
	remainder = remainder - 2.0 * (double)o2;
	o3 = (int)(12.0 * remainder);

	// latitude
	remainder = lat + 90.0;
	a1 = (int)(remainder / 10.0);
	remainder = remainder - (double)a1 * 10.0;
	a2 = (int)(remainder);
	remainder = remainder - (double)a2;
	a3 = (int)(24.0 * remainder);
	GridSquare6Digits[0] = (char)o1 + 'A';
	GridSquare6Digits[1] = (char)a1 + 'A';
	GridSquare6Digits[2] = (char)o2 + '0';
	GridSquare6Digits[3] = (char)a2 + '0';
	GridSquare6Digits[4] = (char)o3 + 'A';
	GridSquare6Digits[5] = (char)a3 + 'A';
	GridSquare6Digits[6] = (char)0;
}

// Read the temp DS18B20 sensor
// Returns temp in Celsius
float GetCurrentTemp(byte* Address)
{
	byte i;
	byte present = 0;
	byte data[9];
	float celsius;

	ds.reset();				// Reset wakes up the bus for a command.

	ds.write(0x55); //Match ROM (Match addr)
	// Send address
	for (int i = 0; i < 8; i++)
	{
		ds.write(Address[i]);
	}
	ds.write(0x44, 1);   // Start conversion, with parasite power on at the end. Result goes to scratchpad memory

	delay(750);     // Wait for the conversion to take place.  750ms is the spec'ed max time needed, give it a little bit more time.

	// Read the scratchpad memory
	present = ds.reset();
	ds.write(0x55); //Match ROM (Match addr)
	// Send address
	for (int i = 0; i < 8; i++)
	{
		ds.write(Address[i]);
	}
	ds.write(0xBE); // Read Scratchpad

	// 9 Bytes of data should be ready to read, so get it.
	for (i = 0; i < 9; i++)
	{
		data[i] = ds.read();
	}

	// Convert the first two bytes to an 16bit int.
	int16_t raw = (data[1] << 8) | data[0];

	// Convert to a float value.
	celsius = (float)raw / 16.0;  // Degrees C
#if DEBUGMESSAGES
	Serial.print(F("Temp=")); Serial.println(celsius);
#endif
	return(celsius);
}


//
// Computes predicted frequency drift due to temperature changes
// Returns 110th of Hz
//
int CalcFreqDriftDelta(float TempC)
{
	//
	//debug
	//
	//return 0;
#if DEBUGMESSAGES
	Serial.print(F("C="));Serial.println(TempC);
#endif
	float TempCSquared = TempC * TempC;

	// Formula derrived from curve fit from test data. Polynomial curve order of 2;

	//  Formula: -3.132026502�10-3�x2�- 5.491572347 x�+ 136.3524712
	double fy = (-0.003132026502*TempCSquared) - (5.491572347*TempC) + 136.3524712;  //Predicted Freq shift (HZ)
#if DEBUGMESSAGES
	Serial.print(F("Calc drift="));Serial.println(fy);
#endif

	
	int iy = fy * SI5351_FREQ_MULT;  // Convert to 100th Hz
#if DEBUGMESSAGES
	Serial.print(F("Drift Delta(hz)="));Serial.println(fy);
#endif

	return iy;

}


//
//  Power Management etc.
//

//
// Get the current battery voltage
//
void GetCurrentVolts()
{
	// Reading the voltage from a 5:1 voltage divider using internal 1.1 v reference.
	// Chips vary as well as the resistors, so hand tune the scale value to match an
	// actual voltage as mesured with a volt meter.
	int v = analogRead(0);
	CurrentVolts = (v / 1023.0) * 6.35; // Scale hand tweaked to get calibrated result matching volt meter.
#if DEBUGMESSAGES
	Serial.print(F("V="));Serial.println(CurrentVolts);
#endif
}

//
// Get the radio board set up and ready to transmit
//
void StartRadio()
{

	if (!bRadioIsOn)  // Only start radio if it is off.
	{
		RadioPower(ON);
		delay(50); // the radio needs a slight "Warm up" time, after power up
		//si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);  //Etherkit Boards
		si5351.init(SI5351_CRYSTAL_LOAD_10PF, 0, 0); // Adafruit boards
		
													 // Set CLK0 output
		si5351.set_correction(CALIBRATIONADJUST); // Measured value from calibration program
		si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
		si5351.set_freq(WSPR_DEFAULT_FREQ, SI5351_CLK0); // Get radio on freq. Use WSPR freq for now.

		si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for max power if desired
		si5351.output_enable(SI5351_CLK0, 0); // Disable the clock initially
#if DEBUGMESSAGES
		Serial.println(F("si5351 Started"));
#endif
	}
	else // Nothing to do, leave radio alone.
	{
#if DEBUGMESSAGES
		Serial.println(F("si5351 already started."));
#endif
	}
	
	
}

//
// Turn Radio Power switch on/off.  
//
void RadioPower(int OnOff)
{
#if DEBUGMESSAGES
	Serial.print(F("R PWR "));Serial.println(OnOff);
#endif
	digitalWrite(RADIOPOWER_PIN, OnOff);
	bRadioIsOn = OnOff;
}

//
// ON THE AIR indicator LED
//
void XmitIndicator(int OnOff)
{
	digitalWrite(XMITLED_PIN, OnOff);
}

//
// Turn GPS power switch on/off.
//
void GPSPower(int OnOff)
{
#if DEBUGMESSAGES
	Serial.print(F("G=")); Serial.println(OnOff);
#endif
	digitalWrite(GPSPOWER_PIN, OnOff);

}


// DEBUG Stuff
#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int freeMemory() {
	char top;
#ifdef __arm__
	return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
	return &top - __brkval;
#else  // __arm__
	return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}
