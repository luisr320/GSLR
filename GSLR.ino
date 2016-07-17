// **********************************************************************************************************
// Moteino GPS Receiver & Telemetry
// 2015-01-09 (C) luisr320@gmail.com
// **********************************************************************************************************
// It receives data from remote Moteino sensors and display data on an Nokia 5110 LCD or TFT
// **********************************************************************************************************
// Creative Commons Attrib Share-Alike License
// You are free to use/extend this code/library but please abide with the CCSA license:
// http://creativecommons.org/licenses/by-sa/3.0/
// **********************************************************************************************************

/*
version V1.3.0
Changes:
date: 2016.07.16 Edited by Luis Rodrigues
- Adafruit 2.4" TFT display with touchscreen added
date: 2016.07.13 Edited by Luis Rodrigues:
- changed the radio handling to LORA with Reliable Datagram
- remote station battery is now displayed
- committed to always using the interrupt for the GPS
date: 2016.07.04 Edited by Luis Rodrigues:
- send to Google function reactivated on menu 2
- removed all bounce2 library references
- added some source references to the libraries include list
- removed all references to the bounce2 library
- cleaned old EEPROM references
date: 2016.07.01 Edited by Pedro Albuquerque
date: 2016.06.29 Edited by Pedro Albuquerque
- GPSMath.h included to allow GPS calculations
date: 2016.06.26 Edited by Luis Rodrigues
date: 2016.06.14 Created by Pedro Albuquerque
-	button speed reaction improvement
To Do:
- include a GPS module on GS to assist on plane rescue
- include a function send a command to activate a buzzer on RS to improve plane detection
*/


#include <RHReliableDatagram.h> //http://www.airspayce.com/mikem/arduino/RadioHead/RadioHead-1.61.zip
//#include <arduino.h> // uncomment if using ATOM editor
#include <SPIFlash.h> //Arduino/Moteino library for read/write access to SPI flash memory chips. https://github.com/LowPowerLab/SPIFlash
#include <SPI.h> //Arduino native SPI library
#include <RH_RF95.h> //Required for the LORA Radios http://www.airspayce.com/mikem/arduino/RadioHead/
#include <Adafruit_GFX.h> //Required for the Nokia 5110 LCD and ST7735 TFT display https://github.com/adafruit/Adafruit-GFX-Library
#include <FlashLogM.h> //To handle the Flash log https://github.com/Pedroalbuquerque/FlashLogM
#include <math.h> //Arduino native math library
#include <GPSMath.h> //To handle all GPS calculations https://github.com/Pedroalbuquerque/GPSMath
#include <Adafruit_GPS.h> //https://github.com/adafruit/Adafruit_GPS/archive/master.zip


//************************* DEFINITIONS ****************************

// Adaruit 2.4 TFT with touchscreen
// For the Moteino Mega, use digital Analog pins A0 through A7
//   D0 connects to analog pin A0
//   D1 connects to analog pin A1
//   D2 connects to analog pin A2
//   D3 connects to analog pin A3
//   D4 connects to analog pin A4
//   D5 connects to analog pin A5
//   D6 connects to analog pin A6
//   D7 connects to analog pin A7
#define YP A0
#define XM A1
#define YM A2
#define XP A3

#define VERSION "GS LR MEGA V1.3.0"
#define FREQUENCY 434 //Match with the correct radio frequency of the other radio
#define SERIAL_BAUD 115200 //To communicate with serial monitor for debug
#define BUTPIN1 12 //Analog pin assigned to FIX button
#define BUTPIN2 13 //Analog pin assigned to MENUS SCROLL button
#define MPAGES 8 //Number of menu pages
#define GOOGLEMAPS //Uncomment to have Google info sent trough the Serial port on menu 2
#define TFT_TOUCH_9341
//#define TFT_ILI9340 //Uncomment to use Adafruit 2.2" TFT display
//#define LCD // uncomment to use NOKIA LCD display
//#define TFT_ST7735 //Uncomment if you use the Seed Studio TFT 1.8"
//#define DEBUG //Uncomment to activate Serial Monitor Debug
#define BUZZER // Comment if a buzzer is installed

#define CLIENT_ADDRESS 2
#define SERVER_ADDRESS 1

//**************************INITIATE HARDWARE*************************

#ifdef BUZZER
	#define BUZZ 14 //Buzzer output pin
#endif

#ifdef __AVR_ATmega1284P__
	#define LED           15 // Moteino MEGAs have LEDs on D15
#else
	#define LED           9 // Moteinos have LEDs on D9
#endif

#ifdef TFT_TOUCH_9341

	#include <Adafruit_TFTLCD.h>
	#include <TouchScreen.h>

	#define YP A0  // must be an analog pin, use "An" notation!
	#define XM A1  // must be an analog pin, use "An" notation!
	#define YM A2  // can be a digital pin
	#define XP A3   // can be a digital pin

	#define TS_MINX 150
	#define TS_MINY 120
	#define TS_MAXX 920
	#define TS_MAXY 940
	
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

	#define LCD_CS 18
	#define LCD_CD 19
	#define LCD_WR 20
	#define LCD_RD 21
	// optional
	#define LCD_RESET 0

	// Assign human-readable names to some common 16-bit color values:
	#define	BLACK   0x0000
	#define	BLUE    0x001F
	#define	RED     0xF800
	#define	GREEN   0x07E0
	#define CYAN    0x07FF
	#define MAGENTA 0xF81F
	#define YELLOW  0xFFE0
	#define WHITE   0xFFFF
	#define ORANGE  0xFC00

	// character size from Adafruit_GXF lib
	#define CHARWIDTH 6
	#define CHARHEIGHT 8
	#define SCRROTATION 0 // 90º rotation
	#define CHARSCALE 2
	#define SCRLINES 10   // 20 Lines or 27 characters / CHARSCALE
	#define SCRCHARS 25   // 21 characters or 16 lines /CHARSCALE

	Adafruit_TFTLCD display(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

	#define BOXSIZE 120
#else

#endif

#ifdef LCD

	#include <Adafruit_PCD8544.h> //Required for the Nokia 5110 LCD display

	// Initiate the display instance - Software SPI (slower updates, more flexible pin options):
	// pin 24/A0 - Serial clock out (SCLK)
	// pin 25/A1 - Serial data out (DIN)
	// pin 26/A2 - Data/Command select (D/C)
	// pin 27/A3 - LCD chip select (CS)
	// pin 28/A4 - LCD reset (RST)

	#define RST  0 //A5 //18
	#define RS   1 //A4 //19
	#define SDA  A2 //20
	#define SCL  A0 //21
	#define CS   3 //A3 //22

	Adafruit_PCD8544 display = Adafruit_PCD8544(SCL, SDA, CS, RS, RST);

	// character size from Adafruit_GXF lib
	#define CHARWIDTH 6
	#define CHARHEIGHT 8
	#define SCRROTATION 0 // 0 rotation
	#define CHARSCALE 1
	#define SCRPIXELX 48
	#define SCRPIXELY 84
	#define SCRLINES 6    // 6 lines
	#define SCRCHARS 14   // 14 characters

	#define PIN_LCD_LIGHT 9 //Backlight pin for NOKIA LCD
#endif

#ifdef TFT_ST7735
	#include <Adafruit_ST7735.h> //Required for OLED LCD
	// pin definition for ITDB02-1.8 TFT display from ITEAD STUDIO
	// assuming the use of Moteino (several pins are reserved)
	#define RST  1 //A5 //18
	#define RS   0 //A4 //19
	#define SDA  A2 //20
	#define SCL  A0 //21
	#define CS   3 //A3 //22
	// Adafruit_ST7735 display = Adafruit_ST7735(CS, RS, SDA, SCL, RST);
	Adafruit_ST7735 display = Adafruit_ST7735(CS, RS, RST);
	//	Adafruit_ST7735 display = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
	#define SCRPIXELX 160
	#define SCRPIXELY 128
	// character size from Adafruit_GXF lib
	#define CHARWIDTH 6
	#define CHARHEIGHT 8
	#define SCRROTATION 1 // 90 deg rotation
	#define CHARSCALE 1
	#define SCRLINES 14   // 20 Lines or 27 characters / CHARSCALE
	#define SCRCHARS 25   // 21 characters or 16 lines /CHARSCALE
	#define BLACK ST7735_BLACK
	#define WHITE ST7735_WHITE
	#define RED ST7735_RED
	#define BLUE ST7735_BLUE
	#define YELLOW ST7735_YELLOW
	#define ORANGE 0xFF00
#endif

#ifdef TFT_ILI9340

	#include <Adafruit_ILI9340_PA.h> //Library required to handle the TFT I2C with the radio interrupts

	// pin definition for FT_ILI9340 ADAFRUIT TFT display
	#define SCL 7 //Clock
	#define MISO 6 //MISO
	#define MOSI 5 //MOSI
	#define CS A3	// A3 //Chip Select
	#define DC A4	//A4 //Data Chip
	#define RST A5	//A5 //Reset

	//Screen size definitions
	#define SCRPIXELX 320
	#define SCRPIXELY 240

	// character size from Adafruit_GXF lib
	#define CHARWIDTH 6
	#define CHARHEIGHT 8
	#define SCRROTATION 1 // 90º rotation
	#define CHARSCALE 2
	#define SCRLINES 14   // 20 Lines or 27 characters / CHARSCALE
	#define SCRCHARS 25   // 21 characters or 16 lines /CHARSCALE

	//Adaruit_ILI9340 display = Adafruit_ILI9340(CS, DC, MOSI, SCL, RST, MISO); //For Software SPI
	Adafruit_ILI9340 display = Adafruit_ILI9340(CS, DC, RST); //For Hardware SPI

	#define BLACK ILI9340_BLACK
	#define WHITE ILI9340_WHITE
	#define RED ILI9340_RED
	#define BLUE ILI9340_BLUE
	#define YELLOW ILI9340_YELLOW
	#define ORANGE 0xFC00
#endif

	//Define GPS information LOG received from RS using SPI Flash Memory
FlashLogM mylog;

//Initialize the generic radio driver instance
RH_RF95 driver;

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SERVER_ADDRESS);

// variables setup
char input = 0;

#ifdef BUZZER
	unsigned long int newbuzztimer;
	unsigned long int oldbuzztimer;
#endif
int rssi; //variable to hold the Radio Signal Strength Indicator
int level = 0;
bool fixinMem = 0; //variable holding fix in memory condition
bool kmflag = 0; //if distance is > 1000m, display in Km
bool kmflagmem = 0; //hold the previous kmflag status
bool button1 = 0;
bool button2 = 0;
byte menuPage = 1; //hold the actual page number on the menu
long int homeazim = 0; //variable to hold azimuth from GPS position to home
long int homealt = 0; //Variable to hold Home altitude (FIX)
long int maxalt = 0; //variable to hold max Altitude
long int homedist = 0; //variable to hold distance to home always in mt
long int maxdist = 0; //variable to hold max distance to home always in mt
float homelat = 0; //variable that will hold the FIX latitude
float homelon = 0; //variable that will hold the FIX longitude
float latmem = 0; //variable that will hold the last latitude memorized
float longmem = 0; //variable that will hold the last longitude memorized
float highspeed = 0; //variable that will hold the highest speed achieved initialized to 0
long major1, minor1;
long major2, minor2;
long longloglat;
long longloglong;
uint8_t oldseconds = 0;
char gpsstr1[80];
char gpsstr2[80];
const byte buff_size = 80; //buffer size must be a constant variable
char buffer[buff_size];
byte index = 0;   //declare all variables that will hold numbers less than '255' as 'byte' data type, because they require only '1-byte' of memory ('int' uses 2-bytes).
byte start_with = 0;
byte end_with = 0;
byte CRC = 0;
boolean data_end = false; //Here we will keep track of EOT (End Of Transmission).


// Define the data packet struct that will be received from the GPS transmitter
struct Payload
{
	char HD[3] = "/*";
	uint8_t hour;
	uint8_t minute;
	uint8_t seconds;
	uint16_t miliseconds;
	uint8_t day;
	uint8_t month;
	uint8_t year;
	float groundspeed; //In knots
	float track; //Course over ground in degrees
	float latitude; //ddmm.mmmm
	char lat; //N/S
	float longitude; //dddmm.mmmm
	char lon; //E/W
	float altitude; //MSL Altitude
	uint8_t fixquality; //Same as 3D FIX
	uint8_t satellites; //Range 0 to 14
	float HDOP; //Horizontal Dilution of Precision <2.0 is good - https://en.wikipedia.org/wiki/Dilution_of_precision_(GPS)
	float geoidheight;
	float latitudedeg;
	float longitudedeg;
	bool fix; // FIX 1/0
	float batteryVolts;
};
Payload Data;

Payload GS_GPS; // struct to hold last good position from GS GPS
Payload RS_GPS; // struct to hold last good position from RS GPS, this cold be the last save position on LOG


// Menu navigation and Warning messages vars and constants

uint8_t warningLevel = 0;	//warning messages are limited to overlap only some menu screens
																										// while warning level != 0 there should be a warning on screen

#define WRN_LINK	1	//b 00000001
#define WRN_FIX		2	//b 00000010
#define WRN_FENCE	4	// ...not yet implemented
#define FLAGSET true
#define FLAGRESET false

																										// Timers
unsigned long int timerLink;	//for data loss timeout calculation
unsigned long int timerWarning = 0; //for warning display @ 2000ms intervals

																																				// General purpose auxiliary vars
char strPRT[100]; // to support any print command with sprintf
char strtmp[40];  // to support float to string conversion or other string manipulation

// object declaration to use GPS

Adafruit_GPS GPS(&Serial1); // connect GPS to serial 1 GPS_TX on pin10 GPS_RX on pin 11
#define GPS_BAUD 9600

// Function declaration if using Visual studio IDE
#define VISUALSTD
#ifdef VISUALSTD
	int len(char* str);
	byte checksum(char *str);
	void changeMenu();
	void displayReset();
	void displaySetCursor(int line, int column);
	void fixposition();
	void logposition(float loglat, float loglong);
	void displaymenu(byte menuPage, bool forceRepaint);
	void displaywarning(int warningcode);
	int len(char *str);
	char* fill(char* str, int length, char charcode, bool initialize);
	uint8_t setflag(uint8_t flagContainer, uint8_t flag, bool set);
	void sendToGoogle(Payload stcData);
#endif


void setup()
{
	// initialize Serial port for debugging
	Serial.begin(SERIAL_BAUD); //Initialize the Serial port at the specified baud rate
	Serial.println F("GPS AND TELEMETRY MODULE");
	Serial.println(VERSION);
	Serial.println F("Initializing...");

	if (manager.init())
	{
		driver.setFrequency(434);
	}
	else
	{
		Serial.println F("init failed");
		// Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
	}

	// initialize GPS
	GPS.begin(GPS_BAUD);
	// Enable interrupts on Timer0 for GPS
	// Timer0 is already used for millis() - we'll just interrupt somewhere
	// in the middle and call the "Compare A" function
	OCR0A = 0xAF;
	TIMSK0 |= _BV(OCIE0A);

	// ### Initialize push-buttons
	pinMode(BUTPIN1, INPUT_PULLUP); // Setup the first button with an internal pull-up
	pinMode(BUTPIN2, INPUT_PULLUP); // Setup the second button with an internal pull-up

		#ifdef LCD
		pinMode(PIN_LCD_LIGHT, OUTPUT); //LCD backlight, LOW = backlight ON
	#endif

	#ifdef BUZZER
		oldbuzztimer = 0; //Initialize buzzer timer variable
		pinMode(BUZZ, OUTPUT); //Make BUZZ pin an output
	#endif

	// SPI initialization
	SPI.usingInterrupt(digitalPinToInterrupt(2));

	// ### Initialize  display

	#ifdef TFT_TOUCH_9341
		display.reset();
		uint16_t identifier = display.readID();
		if (identifier == 0x9341)
		{
			display.begin(identifier);
			display.fillScreen(BLACK);
			display.setTextSize(CHARSCALE);
			display.setRotation(SCRROTATION);
			display.setTextColor(WHITE, BLACK);
			display.setTextWrap(true);
			displaySetCursor(2, 0);
			display.println(F("GPS TELEMETRY"));
			display.println(F("Ver:")); display.println(VERSION);
			delay(3000);
			displaySetCursor(8, 0);
			display.setTextColor(WHITE);  display.setTextSize(2);
			display.println("    Please wait...");
			display.println("   Analizing LOG...");

		}
		else
		{
			Serial.print(F("Unknown LCD driver chip: "));
			Serial.println(identifier, HEX);
			return;
		}
				
		#define MINPRESSURE 10
		#define MAXPRESSURE 1000

	#endif

	#ifdef LCD // display SETUP (Nokia LCD)
		display.begin();
		displayReset(); //Cleanup the LCD
		display.setContrast(60); // you can change the contrast around to adapt the display for the best viewing!
		display.println("Starting LCD display!");
		display.print("Ver. "); display.println(VERSION);
		display.display(); // show splash screen
		delay(1000);
	#endif

	#ifdef TFT_ST7735	// display Setup OLED LCD
		display.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
		// POSITION CURSOR ON TOP LEFT
		displayReset();
		display.println("Starting TFT display!");
		display.print("Ver:"); display.println(VERSION);
		delay(5000);
	#endif

	#ifdef TFT_ILI9340 // display Setup OLED LCD
		display.begin();
		displayReset();
		display.println("Starting TFT display!");
		display.print("Ver:"); display.println(VERSION);
		Serial.println("TFT Display ILI9340 - initialized");
	#endif

	// ### Initialize Log

	// initialize log variables to start write and read
	mylog.initialize(Data);
	Serial.println F("Log data initialized");
	Serial.print F("Log next write addr:"); Serial.println(mylog.nextWrite);
	Serial.print F("Log next read addr:"); Serial.println(mylog.nextRead);
	Serial.print F("Log # records saved:"); Serial.println(mylog.numRecords);

	// load menu screen on LCD/TFT
	displaymenu(menuPage, false); //Start menu display (menu page number, screen refresh requirement)

	Serial.println F("Setup finished");

	timerLink = millis(); //Initialize Data link loss timeout timer variable
}

void loop()
{
	makeHeader();
	#ifdef TFT_TOUCH_9341

		TSPoint p = ts.getPoint();
		pinMode(XM, OUTPUT);
		pinMode(YP, OUTPUT);

		if (p.z > MINPRESSURE && p.z < MAXPRESSURE)
		{
		
			// scale from 0->1023 to tft.width
			p.x = (map(p.x, TS_MINX, TS_MAXX, display.width(), 0)) - 5;
			p.y = (map(p.y, TS_MINY, TS_MAXY, display.height(), 0)) - 15;

			if (p.y > 200)
			{
				if (p.x < BOXSIZE) {
					#ifdef BUZZER
						Blink(BUZZ, 5);
					#endif
						button1 = 1;
				}
				else if (p.x < BOXSIZE * 2) {
					button2 = 1;
				}
			}
		}
	#endif

	if (!digitalRead(BUTPIN1))  // process button 1 if pressed - Menu navigation
		button1 = 1;
	else if (!digitalRead(BUTPIN2))
		button2 = 1;

	
	if (button1 == 1)  // process button 1 if pressed - Menu navigation
	{
		button1 = 0;
		changeMenu();
		displaymenu(menuPage, true);
		warningLevel = setflag(warningLevel, 0xFF, FLAGRESET); // reset all warning to force re-evaluation
	}
	if (button2 == 1)// process button 2 if pressed - Function within Menu
	{
		button2 = 0;
		switch (menuPage)
		{
		case 1: // Navigate
		#ifdef BUZZER
			Blink(BUZZ, 5);
		#endif
			fixposition();
			break;
		case 2: // GPS info

										//No action for B2
			break;
		case 3: // Maximum

										//No action for B2
			break;
		case 4: // Coordinates

										//No action for B2
			break;
		case 5: // Model recovery

										//No action for B2
			break;
		case 6: // Utils Adjust LCD brightness
			#ifdef LCD
				level = level + 51;
				if (level > 255) level = 0;
				analogWrite(PIN_LCD_LIGHT, level);
			#endif
			break;
		case 7: // LOG erase
			#ifdef BUZZER
				Blink(BUZZ, 10);
			#endif
			displaySetCursor(1, 0); display.print(fill(strPRT, SCRCHARS, ' ', true));
			displaySetCursor(2, 0); display.print(fill(strPRT, SCRCHARS, ' ', true));
			displaySetCursor(3, 0); display.print(fill(strPRT, SCRCHARS, ' ', true));
			displaySetCursor(2, 0); sprintf(strPRT, "ERASING LOG..."); display.print(strPRT);
			mylog.eraseData(); //erase LOG memory on B2
			displaymenu(menuPage, true);
			break;
		case 8: // dump log to Googlemaps
			//  display some activity message
			#ifdef BUZZER
				Blink(BUZZ, 10);
			#endif
			displaySetCursor(1, 0); display.print(fill(strPRT, SCRCHARS, ' ', true));
			displaySetCursor(2, 0); display.print(fill(strPRT, SCRCHARS, ' ', true));
			displaySetCursor(3, 0); display.print(fill(strPRT, SCRCHARS, ' ', true));
			displaySetCursor(2, 0); sprintf(strPRT, "Dumping LOG..."); display.print(strPRT);
			uint16_t logStart = mylog.nextRead;
			Payload logData;

			Serial.println F("\n********   Log dump ********");
			// Read data from log and send it to Google as data is read
			noInterrupts(); // so that no additional log data is saved
			for (uint16_t i = 1; i < mylog.numRecords; i++)
			{
				mylog.readData(logData);
				sendToGoogle(logData);
			}
			mylog.nextRead = logStart;	// reposition nextRead pointer for next read if necessary
			interrupts();
			displaymenu(menuPage, true);
			break;
		}
		#ifdef LCD
			display.display();
		#endif
		warningLevel = setflag(warningLevel, 0xFF, FLAGRESET); // reset all warning to force re-evaluation
	}
	

	// check radio reception and process if data is available
	if (timerLink > millis()) timerLink = millis();
	if (manager.available()) //If some packet was received by the radio, wait for all its contents to come trough
	{
		warningLevel = setflag(warningLevel, WRN_LINK, FLAGRESET); // clear Link flag

		uint8_t len = sizeof(Data);
		uint8_t from;

		if (manager.recvfromAck((uint8_t *)&Data, &len, &from))

		{
			timerLink = millis(); //Set a counter for data link loss timeout calculation

	
			rssi = driver.lastRssi();	//RSSI;


			if (Data.fix == 1)	// if GPS fix acquired
			{
				if (fixinMem == 0) fixposition();
					fixinMem = 1;

				warningLevel = setflag(warningLevel, WRN_FIX, FLAGRESET);

				// Run the distance calculating function, passing the memorized position as arguments
				kmflag = GPSDist(homelat, homelon, Data.latitudedeg, Data.longitudedeg, &homedist, &homeazim); 


				if (Data.altitude > maxalt + homealt)	// check maximum altitude
					maxalt = Data.altitude - homealt;

				if (kmflag == 0)	// check maximum distance
				{
					kmflagmem = kmflag;
					if (homedist > maxdist)
						maxdist = homedist;
				}
				else
				{
					if (kmflagmem == 0)
					{
						kmflagmem = 1;
						maxdist = homedist;
					}
					else if (homedist > maxdist)
						maxdist = homedist;
				}

				//check maximum speed
				if (Data.groundspeed*1.852 < 1)
					Data.groundspeed = 0;
				if (Data.groundspeed*1.852 > highspeed)
					highspeed = Data.groundspeed*1.852;
			}
			else
				warningLevel = setflag(warningLevel, WRN_FIX, FLAGSET);

			// save data just received to Log memory
			mylog.saveData(Data);
			#ifdef GOOGLEMAPS
				sendToGoogle(Data);
			#endif
			displaymenu(menuPage, false); // update menu info (menu page number, screen refresh)
		}
	}
	else // if no data received
	{
		// check if link timeout, ie, no data received for more than 5 sec
		if (millis() > (timerLink + 5000)) //If LINK lost for more than 5 sec...
		{
			warningLevel = setflag(warningLevel, WRN_LINK, FLAGSET);  // set LINK flag
			#ifdef BUZZER
				newbuzztimer = millis();
				if (newbuzztimer > (oldbuzztimer + 2000))
				{
					oldbuzztimer = newbuzztimer;
					Blink(BUZZ, 5);
					delay(100);
					Blink(BUZZ, 5);
				}
			#endif
		}
	}
	// check Warnings
	if (millis() < timerWarning)
		timerWarning = millis(); // if millis() wrap around reinitialize timer
	if (millis() > timerWarning + 2000)
	{
		displaywarning(warningLevel);
		timerWarning = millis();
	}

	/*
	// check GPS data if on Search menu (menu 9)
	if(menuPage = 9)
	{
		// place here code to process local GPS data, calculate distance and azimuth from last good RS position
		if (GPS.newNMEAreceived())
		{
			if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
			{
				Serial.println("GS NMEA not parsed");
				return;                         // we can fail to parse a sentence in which case we should just wait for another
			}
			else
			{
				// save local GPS data to a var
			}

			// put here code to calculate distance and azimuth from GS to last RS good location
		}

	}
	*/

}

//----------------------------------------------------------------------//
//                             FUNCTIONS                                //
//----------------------------------------------------------------------//

void makeHeader()
{
	if (Data.batteryVolts < 3.2)
	{
		display.fillRect(204, 0, 14, 12, RED);
		display.drawRect(204, 0, 14, 12, BLACK);
		display.fillRect(216, 0, 14, 12, BLACK);
		display.drawRect(216, 0, 14, 12, BLACK);
		display.fillRect(228, 0, 14, 12, BLACK);
		display.drawRect(228, 0, 14, 12, BLACK);
	}
	else if (Data.batteryVolts >= 3.2 & Data.batteryVolts < 3.6)
	{
		display.fillRect(204, 0, 14, 12, YELLOW);
		display.drawRect(204, 0, 14, 12, BLACK);
		display.fillRect(216, 0, 14, 12, YELLOW);
		display.drawRect(216, 0, 14, 12, BLACK);
		display.fillRect(228, 0, 14, 12, BLACK);
		display.drawRect(228, 0, 14, 12, BLACK);
	}
	else
	{
		display.fillRect(204, 0, 14, 12, GREEN);
		display.drawRect(204, 0, 14, 12, BLACK);
		display.fillRect(216, 0, 14, 12, GREEN);
		display.drawRect(216, 0, 14, 12, BLACK);
		display.fillRect(228, 0, 14, 12, GREEN);
		display.drawRect(228, 0, 14, 12, BLACK);
	}
	display.setCursor(90, 0);
	if (Data.hour < 10)
		display.print("0");
	display.print(Data.hour);
	display.print(":");
	if (Data.minute < 10)
		display.print("0");
	display.print(Data.minute);
	display.drawLine(0, 20, 240, 20, RED);
}


uint8_t setflag(uint8_t flagContainer, uint8_t flag, bool set)
{
	if (set) return (flagContainer | flag); else return (flagContainer & ~flag);
}

byte checksum(char *str)
{
	byte sum, l;
	sum = 0;
	l = len(str);
	for (int i = 0; i < l; i++)
	{
		sum ^= str[i];
	}
	return sum;
}

void changeMenu()
{
	//  Blink(BUZZ, 50);
	menuPage = menuPage + 1;
	if (menuPage > MPAGES)
	{
		menuPage = 1;;
	}
}

#ifdef BUZZER
void Blink(byte PIN, int DELAY_MS)//The BUZZ Blinking function
{
	digitalWrite(PIN, HIGH);
	delay(DELAY_MS);
	digitalWrite(PIN, LOW);
}
#endif

	void displayReset()
	{
		display.fillScreen(BLACK);

		display.fillRect(0, 200, BOXSIZE, BOXSIZE, RED);
		display.drawRect(0, 200, BOXSIZE, BOXSIZE, WHITE);

		display.fillRect(BOXSIZE, 200, BOXSIZE, BOXSIZE, YELLOW);
		display.drawRect(BOXSIZE, 200, BOXSIZE, BOXSIZE, WHITE);

		displaySetCursor(16, 4);
		display.setTextColor(WHITE);  display.setTextSize(4);
		display.println("B1");

		displaySetCursor(16, 14);
		display.setTextColor(BLUE);  display.setTextSize(4);
		display.println("B2");


		display.setTextSize(CHARSCALE);
		display.setRotation(SCRROTATION);
		display.setTextColor(WHITE, BLACK);
		display.setTextWrap(true);
		display.setCursor(0, 0);
		return;
	}


void displaySetCursor(int line, int column)
{
	display.setCursor(column * CHARWIDTH * CHARSCALE, line * CHARHEIGHT*CHARSCALE);
	return;
}

void fixposition()
{
	highspeed = 0;//Reset maximum speed
	maxdist = 0; //Reset maximum distance
	maxalt = 0; //Reset maximum altitude
	kmflagmem = 0; //Reset Km flag mem
	homeazim = 0;
	homelat = Data.latitudedeg;//Memorize FIX latitude in DDMM.SS
	homelon = Data.longitudedeg;//Memorize FIX longitude in DDDMM.SS
	displayReset();
	display.println("FIX Position Memorized");
	display.println("LAT:"); display.println(homelat, 8); display.println("LON:"); display.println(homelon, 8);
	homealt = Data.altitude;
	#ifdef LCD
		display.display();
	#endif
	delay(2000);
	displaymenu(menuPage, true);
}

void logposition(float loglat, float loglong)
{
	longloglat = loglat * 1000000;
	longloglong = loglong * 1000000;
}

void displaymenu(byte menuPage, bool forceRepaint)
{

	static byte lastmenu;  //remember last menu for paint or refresh

	switch (menuPage)
	{
		case 1: //NAVIGATION STATUS MENU
		{
			if (lastmenu != menuPage || forceRepaint)
			{
				lastmenu = menuPage;
				displayReset();
				displaySetCursor(2, 0);
				display.print("1 - STATUS");
				displaySetCursor(3, 0); sprintf(strPRT, "SAT:%i", Data.satellites); display.print(strPRT);
				displaySetCursor(4, 0); sprintf(strPRT, "QUAL:%d", Data.fixquality); display.print(strPRT);
				displaySetCursor(5, 0); sprintf(strPRT, "HDOP:%s", dtostrf(Data.HDOP, 5, 2, strtmp)); display.print(strPRT);
				displaySetCursor(6, 0); sprintf(strPRT, "RX_RSSI:%d", rssi); display.print(strPRT);
				displaySetCursor(7, 0); sprintf(strPRT, "RS BAT:%s V", dtostrf(Data.batteryVolts, 4, 2, strtmp)); display.print(strPRT);
			}
			else // if already in menu, print just info to reduce screen flicker
			{
				displaySetCursor(3, 4); 
				display.print(Data.satellites);
				displaySetCursor(4, 5);
				display.print(Data.fixquality);
				displaySetCursor(5, 5);  sprintf(strPRT, "%s", dtostrf(Data.HDOP, 5, 2, strtmp)); display.print(strPRT);
				displaySetCursor(6, 8);	display.print(rssi);
				displaySetCursor(7, 7); sprintf(strPRT, "%s", dtostrf(Data.batteryVolts, 4, 2, strtmp));	display.print(strPRT);
			}
			break;
		}

		case 2: //INFO MENU
		{
			if (lastmenu != menuPage || forceRepaint)
			{
				lastmenu = menuPage;
				displayReset();
				displaySetCursor(2, 0); display.print("2 - INFO");
				sprintf(strPRT, "ALT:%s m", dtostrf(Data.altitude - homealt, 4, 0, strtmp)); displaySetCursor(3, 0); display.print(strPRT);
				sprintf(strPRT, "SPD:%s Km/h", dtostrf(Data.groundspeed*1.852, 4, 0, strtmp)); displaySetCursor(4, 0); display.print(strPRT);
				sprintf(strPRT, "AZM:%s Deg", dtostrf(homeazim, 4, 0, strtmp)); displaySetCursor(5, 0); display.print(strPRT);
				displaySetCursor(6, 0); sprintf(strPRT, "DST:%s", dtostrf(homedist, 4, 0, strtmp)); display.print(strPRT);
				if (kmflag == 0) display.print(" m "); else display.print(" Km");
				display.setTextSize(CHARSCALE + 2);
				displaySetCursor(8, 0); sprintf(strPRT, "%s Km/h", dtostrf(highspeed, 1, 0, strtmp)); display.print(strPRT);
				display.setTextSize(CHARSCALE);
			}
			else
			{
				sprintf(strPRT, "%s m      ", dtostrf(Data.altitude - homealt, 4, 0, strtmp)); displaySetCursor(3, 4); display.print(strPRT);
				sprintf(strPRT, "%s Km/h      ", dtostrf(Data.groundspeed*1.852, 4, 0, strtmp)); displaySetCursor(4, 4); display.print(strPRT);
				sprintf(strPRT, "%s Deg      ", dtostrf(homeazim, 4, 0, strtmp)); displaySetCursor(5, 4); display.print(strPRT);
				displaySetCursor(6, 4); sprintf(strPRT, "%s", dtostrf(homedist, 4, 0, strtmp)); display.print(strPRT);
				if (kmflag == 0) display.print(" m "); else display.print(" Km"); //Spaces added to allow "m" or "Km" to be erased after large numbers
				display.setTextSize(CHARSCALE + 2);
				displaySetCursor(8, 0); sprintf(strPRT, "%s Km/h", dtostrf(highspeed, 1, 0, strtmp)); display.print(strPRT);
				display.setTextSize(CHARSCALE);
			}
			break;
		}

		case 3: //MAXIMUM MENU
		{
			if (lastmenu != menuPage || forceRepaint)
			{
				lastmenu = menuPage;
				displayReset();
				displaySetCursor(2, 0); display.print("3 - MAXIMUM");
				displaySetCursor(3, 0); sprintf(strPRT, "MxSpd:%s K/h", dtostrf(highspeed, 4, 0, strtmp)); display.print(strPRT);
				displaySetCursor(4, 0); sprintf(strPRT, "MxAlt:%4d m", maxalt); display.print(strPRT);
				displaySetCursor(5, 0); sprintf(strPRT, "MxDst:%4d", maxdist); display.print(strPRT);
				if (kmflag == 0) display.print(" m "); else display.print(" Km");
				displaySetCursor(7, 0); sprintf(strPRT, "GPS Alt:%s m", dtostrf(Data.altitude, 5, 0, strtmp)); display.print(strPRT);
			}
			else
			{
				displaySetCursor(3, 6); sprintf(strPRT, "%s", dtostrf(highspeed, 4, 0, strtmp)); display.print(strPRT);
				displaySetCursor(4, 6); sprintf(strPRT, "%4d", maxalt); display.print(strPRT);
				displaySetCursor(5, 6); sprintf(strPRT, "%4d", maxdist); display.print(strPRT);
				if (kmflag == 0) display.print(" m "); else display.print(" Km");
				displaySetCursor(7, 8); sprintf(strPRT, "%s ", dtostrf(Data.altitude, 5, 0, strtmp)); display.print(strPRT);
			}
			break;
		}

		case 4: //COORDINATES MENU
		{
			if (lastmenu != menuPage || forceRepaint)
			{
				lastmenu = menuPage;
				displayReset();
				displaySetCursor(2, 0); display.print("4 - P. POS");
				displaySetCursor(3, 0); sprintf(strPRT, "LAT:%c %s", Data.lat, dtostrf(Data.latitude, 9, 4, strtmp)); display.print(strPRT);
				displaySetCursor(4, 0); sprintf(strPRT, "LON:%c %s", Data.lon, dtostrf(Data.longitude, 9, 4, strtmp)); display.print(strPRT);
				displaySetCursor(5, 0); sprintf(strPRT, "GOOGLE COORD:"); display.print(strPRT);
				displaySetCursor(6, 0); sprintf(strPRT, "%s", dtostrf(Data.latitudedeg, 10, 6, strtmp)); display.print(strPRT);
				displaySetCursor(7, 0); sprintf(strPRT, "%s", dtostrf(Data.longitudedeg, 10, 6, strtmp)); display.print(strPRT);
			}
			else
			{
				displaySetCursor(3, 4); sprintf(strPRT, "%c %s", Data.lat, dtostrf(Data.latitude, 9, 4, strtmp)); display.print(strPRT);
				displaySetCursor(4, 4); sprintf(strPRT, "%c %s", Data.lon, dtostrf(Data.longitude, 9, 4, strtmp)); display.print(strPRT);
				displaySetCursor(6, 0); sprintf(strPRT, "%s", dtostrf(Data.latitudedeg, 10, 6, strtmp)); display.print(strPRT);
				displaySetCursor(7, 0); sprintf(strPRT, "%s", dtostrf(Data.longitudedeg, 10, 6, strtmp)); display.print(strPRT);
			}
			break;
		}

		case 5: //RECOVERY MENU
		{
			if (lastmenu != menuPage || forceRepaint)
			{
				lastmenu = menuPage;
				displayReset();
				displaySetCursor(2, 0); display.print("5 - RECOVERY");
				displaySetCursor(3, 0); display.print("GOOGLE COORD:");
				displaySetCursor(4, 0); sprintf(strPRT, "%s", dtostrf(Data.latitudedeg, 9, 6, strtmp)); display.print(strPRT);
				displaySetCursor(5, 0); sprintf(strPRT, "%s", dtostrf(Data.longitudedeg, 9, 6, strtmp)); display.print(strPRT);
				displaySetCursor(6, 0); sprintf(strPRT, "AZM:%d", homeazim); display.print(strPRT);
				displaySetCursor(7, 0); sprintf(strPRT, "DIS:%d", homedist); display.print(strPRT);
				if (kmflag == 0) display.print("m"); else display.print("Km");
			}
			else
			{
				displaySetCursor(4, 0); sprintf(strPRT, "%s", dtostrf(Data.latitudedeg, 9, 6, strtmp)); display.print(strPRT);
				displaySetCursor(5, 0); sprintf(strPRT, "%s", dtostrf(Data.longitudedeg, 9, 6, strtmp)); display.print(strPRT);
				displaySetCursor(6, 4); sprintf(strPRT, "%d", homeazim); display.print(strPRT);
				displaySetCursor(7, 4); sprintf(strPRT, "%d", homedist); display.print(strPRT);
				if (kmflag == 0) display.print("m"); else display.print("Km");
			}
			break;
		}

		case 6: //TOOLS MENU
		{
			if (lastmenu != menuPage || forceRepaint)
			{
				lastmenu = menuPage;

				displayReset();
				displaySetCursor(2, 0); display.print("6 - TOOLS");
				displaySetCursor(3, 0); display.print("Curr. Ver.:");
				displaySetCursor(4, 0); sprintf(strPRT, "%s", VERSION); display.print(strPRT);
			}
			break;
		}

		case 7: //LOG MEN
		{
			if (lastmenu != menuPage || forceRepaint)
			{
				lastmenu = menuPage;

				displayReset();
				displaySetCursor(2, 0); display.print("7 - LOGGER");
				displaySetCursor(3, 0); sprintf(strPRT, "#Records:%lu", mylog.numRecords); display.print(strPRT);
				displaySetCursor(4, 0); sprintf(strPRT, "press B2 to erase!"); display.print(strPRT);
			}
			else
			{
				displaySetCursor(3, 9); sprintf(strPRT, "%lu", mylog.numRecords); display.print(strPRT);
			}
			break;
		}
		case 8: //LOG dump
		{
			if (lastmenu != menuPage || forceRepaint)
			{
				lastmenu = menuPage;

				displayReset();
				displaySetCursor(2, 0); display.print("8 - Log Dump");
				displaySetCursor(3, 0); sprintf(strPRT, "connect to PC", mylog.numRecords); display.print(strPRT);
				displaySetCursor(4, 0); sprintf(strPRT, "press B2 to dump!"); display.print(strPRT);
			}
			break;
		}
	}
	#ifdef LCD
		display.display();
	#endif
	return;
}

void displaywarning(int warningcode)
{
	displaySetCursor(SCRLINES, 0);
	display.setTextColor(WHITE, BLACK);
	display.print(fill(strtmp, SCRCHARS, ' ', true));
	displaySetCursor(SCRLINES, 0);
	if (warningcode & WRN_LINK) //link lost
	{
		#ifdef DEBUG
			Serial.print F("link Lost");
		#endif

		display.setTextColor(WHITE, RED);
		display.print(" LNK ");
	}
	if (warningcode & WRN_FIX)  // GPS fix lost
	{
		#ifdef DEBUG
			Serial.println F("GPS fix Lost");
		#endif

		display.setTextColor(BLACK, ORANGE);
		display.print(" GPS ");

	}
	displaySetCursor(0, 0);
	display.setTextColor(WHITE, BLACK);

	#ifdef LCD
		display.display();
	#endif
}

void sendToGoogle(Payload stcData)
{
	int latint = (int)stcData.latitude;
	int latdec = (stcData.latitude * 10000) - (latint * 10000);
	int lonint = (int)stcData.longitude;
	int londec = (stcData.longitude * 10000) - (lonint * 10000);

	// Convert altitude to a string
	char falt[8];
	dtostrf(stcData.altitude, 4, 1, falt);

	// Convert speed to a string
	char fspeed[8];
	dtostrf(stcData.groundspeed, 4, 2, fspeed);

	// Convert track to a string
	char ftrack[8];
	dtostrf(stcData.track, 4, 2, ftrack);

	// Convert HDOP to a string
	char fHDOP[4];
	dtostrf(stcData.HDOP, 1, 2, fHDOP);

	// Convert geoid height to a string
	char fgeoh[8];
	dtostrf(stcData.geoidheight, 4, 1, fgeoh);

	//$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
	char *j = gpsstr1;
	j += sprintf(j, "GPGGA,");
	j += sprintf(j, "%.2d%.2d%.2d.000,", stcData.hour, stcData.minute, stcData.seconds, stcData.miliseconds); //123519 Fix taken at 12:35 : 19 UTC
	j += sprintf(j, "%.4d.%.4d,%c,", latint, latdec, stcData.lat); // 4807.038,N Latitude 48 deg 07.038' N
	j += sprintf(j, "%.5d.%.4d,%c,", lonint, londec, stcData.lon); // 4807.038,N Latitude 48 deg 07.038' N
	j += sprintf(j, "1,");                    //   1 Fix quality : 1 - Must always be 1 or we wouldn't be here
	j += sprintf(j, "%.2d,", stcData.satellites);        //   08           Number of satellites being tracked
	j += sprintf(j, "%s,", fHDOP);
	j += sprintf(j, "%s,M,", falt);       //   545.4, M      Altitude, Meters, above mean sea level
	j += sprintf(j, "%s,M,,", fgeoh); // Geoid height

	char hexCS1[2];
	sprintf(hexCS1, "%02X", checksum(gpsstr1));
	Serial.print F("$"); Serial.print(gpsstr1); Serial.print F("*"); Serial.println(hexCS1);

	//$GPRMC,233913.000,A,3842.9618,N,00916.8614,W,0.50,50.58,180216,,,A*4A
	char *k = gpsstr2;
	k += sprintf(k, "GPRMC,");
	k += sprintf(k, "%.2d%.2d%.2d.000,", stcData.hour, stcData.minute, stcData.seconds, stcData.miliseconds);
	k += sprintf(k, "A,"); // A = OK
	k += sprintf(k, "%.4d.%.4d,%c,", latint, latdec, stcData.lat); // 4807.038,N Latitude 48 deg 07.038' N
	k += sprintf(k, "%.5d.%.4d,%c,", lonint, londec, stcData.lon); // 4807.038,N Latitude 48 deg 07.038' N
	k += sprintf(k, "%s,", fspeed);
	k += sprintf(k, "%s,", ftrack);
	k += sprintf(k, "%.2d%.2d%.2d,,,A", stcData.day, stcData.month, stcData.year);

	char hexCS2[2];
	sprintf(hexCS2, "%02X", checksum(gpsstr2));
	Serial.print F("$"); Serial.print(gpsstr2); Serial.print F("*"); Serial.println(hexCS2);
}

/*
*  utility function for string ( char *) manipulation
*/

int len(char *str)
{
	int i = 0;
	while (str[i++] != 0);
	return i - 1;
}

char* fill(char* str, int length, char charcode, bool initialize)
{
	int i = 0;
	if (!initialize) i = len(str);  // start from were str is already filled
	while (i<length)
	{
		str[i++] = charcode;
	}
	str[i] = NULL;
	return str;
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect)
{
		GPS.read();
}
