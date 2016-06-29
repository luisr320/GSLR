
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
* version 1.2.0
* 
* date 2016.06.29 Edited by Pedro Albuquerque
* Requirements:
*	GPSMath.h
*/

/*
* version 0.4 RC01
* date:2016.06.14 Created by Pedro Albuquerque
* date 2016.06.26 Edited by Luis rodrigues

	This is the first version adapted for LORA version of the Moteino and Moteino MEGA.
	Three major changes have occurred and as such, serious testing has to be done before accepting it as good
	Major changes are:
	1-porting code to Moteino LORA radios and MEGA processor
	2-activating received data LOG using external Flash memory (4Mbit)
	3-use TFT display using SPI interface

	To Do:
	- fix the distance calculation routine, that is show inconsistent results
	- debug situation were no warning is displayed. WarningLevel var is consistent with status, but not displayed on display
	- debug slow button reaction when radio data is being received.
*/


#define VERSION "GS LR MEGA 0.4 - RC01"


//#include <arduino.h>
#include <SPIFlash.h>
#include <SPI.h>
#include <RH_RF95.h> //Required for the LORA Radios
#include <Adafruit_GFX.h> //Required for the Nokia 5110 LCD and ST7735 TFT display
#include <Bounce2.h> //To handle buttons debounce
#include <FlashLogM.h> //To handle the Flash log
#include <math.h>
#include <GPSMath.h>
//#include "PA_STR.h"

//************************* DEFINITIONS ****************************
#define FREQUENCY 434 //Match with the correct radio frequency of the other radio
#define SERIAL_BAUD 115200 //To comunicate with serial monitor for debug
#define BUTPIN1 A7 //Analog pin assigned to FIX button
#define BUTPIN2 A6 //Analog pin assigned to MENUS SCROLL button
#define MPAGES 7 //Number of menu pages
#define GOOGLEMAPS //Uncomment to have google info sent trough the Serial port on menu 2
//#define TFT_ILI9340 //Uncomment to use Adafruit 2.2" TFT display
#define TFT_ST7735 // comment if using other display
//#define DEBUG //Uncomment to activate Serial Monitor Debug
//#define EEPROM_SIZE 1024
//#define FLASH_SS 8 // FLASH SS enable pin is on D8
//#define FLASH_MAXADR  524288  // SPI flash = 4Mb = 512KB = 524288 B
//#define BUZZER // Comment if a buzzer is installed
//#define LCD //Uncomment if you use the NOKIA LCD
//#define TFT_ST7735 //Uncomment if you use the Seed Studio TFT


//**************************INITIATE HARDWARE*************************

#ifdef BUZZER
	#define BUZZ A6 //Buzzer output pin
#endif

#ifdef __AVR_ATmega1284P__
	#define LED           15 // Moteino MEGAs have LEDs on D15
#else
	#define LED           9 // Moteinos have LEDs on D9
#endif

#ifdef LCD

	#include <Adafruit_PCD8544.h> //Required for the Nokia 5110 LCD display
	
// Initiate the display instance - Software SPI (slower updates, more flexible pin options):
	// pin 24/A0 - Serial clock out (SCLK)
	// pin 25/A1 - Serial data out (DIN)
	// pin 26/A2 - Data/Command select (D/C)
	// pin 27/A3 - LCD chip select (CS)
	// pin 28/A4 - LCD reset (RST)
	Adafruit_PCD8544 display = Adafruit_PCD8544(A0, A1, A2, A3, A4);

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
	// assuming the use of moteino (several pins are reserved)
	#define RST  A5 //18
	#define RS   A4 //19
	#define SDA  A2 //20
	#define SCL  A0 //21
	#define CS   A3 //22

	// Adafruit_ST7735 display = Adafruit_ST7735(CS, RS, SDA, SCL, RST);
	Adafruit_ST7735 display = Adafruit_ST7735(CS, RS,  RST);
	//	Adafruit_ST7735 display = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

	#define SCRPIXELX 160
	#define SCRPIXELY 128
	// character size from Adafruit_GXF lib
	#define CHARWIDTH 6
	#define CHARHEIGHT 8
	#define SCRROTATION 1 // 90 deg rotation
	#define CHARSCALE 2
	#define SCRLINES 7   // 20 Lines or 27 charactares / CHARSCALE
	#define SCRCHARS 13   // 21 characters or 16 lines /CHARSCALE



	#define BLACK ST7735_BLACK
	#define WHITE ST7735_WHITE
	#define RED ST7735_RED
	#define BLUE ST7735_BLUE
	#define YELLOW ST7735_YELLOW
	#define ORANGE 0xFF00

#endif

#ifdef TFT_ILI9340
	#include "Adafruit_ILI9340.h" //Library required to handle the TFT I2C with the radio interrupts
		
	// pin definition for FT_ILI9340 ADAFRUIT TFT display
	#define SCL A0 //Clock
	#define MISO A1 //MISO
	#define MOSI A2 //MOSI
	#define CS A3 //Chip Select
	#define DC A4 //Data Chip
	#define RST A5 //Reset

	//Screen size defenition
	#define SCRPIXELX 320
	#define SCRPIXELY 240

	// character size from Adafruit_GXF lib
	#define CHARWIDTH 6
	#define CHARHEIGHT 8
	#define SCRROTATION 1 // 90º rotation
	#define CHARSCALE 2
	#define SCRLINES 14   // 20 Lines or 27 charactares / CHARSCALE
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

//Initialize the radio instance
RH_RF95 radio;

//Initialize press buttons instances B1 and B2	and instantiate a Bounce object for each button
Bounce debouncer1 = Bounce();
Bounce debouncer2 = Bounce();

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
byte menuPage = 1; //hold the actual page number on the menu
long int homeazim = 0; //variable to hold azimute from GPS position to home
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

const byte buff_size = 80; // buffer size must be a constant variable
char buffer[buff_size];
byte index = 0;   // declare all variables that will hold numbers less than '255' as 'byte' data type, because they require only '1-byte' of memory ('int' uses 2-bytes).
byte start_with = 0;
byte end_with = 0;
byte CRC = 0;
boolean data_end = false; // Here we will keep track of EOT (End Of Transmission).


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
	float groundspeed; // In knots
	float track; // Course over ground in degrees
	float latitude; //ddmm.mmmm
	char lat; // N/S
	float longitude; // dddmm.mmmm
	char lon; // E/W
	float altitude; //  MSL Altitude
	uint8_t fixquality; // Same as 3D FIX
	uint8_t satellites; // Range 0 to 14
	float HDOP; // Horizontal Dilution of Precision <2.0 is good - https://en.wikipedia.org/wiki/Dilution_of_precision_(GPS)
	float geoidheight;
	float latitudedeg;
	float longitudedeg;
	bool fix; // FIX 1/0
};
Payload Data;

// Menu navigation and Warning messages vars and constants

uint8_t warningLevel = 0;	//warning messages are limited to overlap only some menu screens
// while warning level != 0 there should be a warning on screen

#define WRN_LINK	1	//b 00000001
#define WRN_FIX		2	//b 00000010
#define WRN_FENCE	4	// ...not yet implemented
#define FLAGSET true
#define FLAGRESET false

// Timers
unsigned long int timerLink;	//for dataloss timeout calculation
unsigned long int timerWarning  = 0; //for warning display @ 2000ms intervals 

// General purpose auxiliar vars
char strPRT[100]; // to support any print command with sprintf
char strtmp[40];  // to suport float to string convertion or other string manipulation

// Function declaration because Visual studio is demanding!!! HUUUGGHHHH
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
	void sendToGoogle(Payload strData);
#endif

void setup()
{

	Serial.begin(SERIAL_BAUD); //Initialize the Serial port at the specified baud rate
	Serial.println("GPS AND TELEMETRY MODULE");
	Serial.println(VERSION);

	Serial.println("Initializing...");

	// ### Initialize push-buttons
	pinMode(BUTPIN1, INPUT_PULLUP);	// Setup the first button with an internal pull-up
	debouncer1.attach(BUTPIN1);	// After setting up the button, setup the Bounce instance
	debouncer1.interval(5); // Debounce interval in ms

	pinMode(BUTPIN2, INPUT_PULLUP);	// Setup the second button with an internal pull-up
	debouncer2.attach(BUTPIN2);	// After setting up the button, setup the Bounce instance
	debouncer2.interval(5); // Debounce interval in ms

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
	#ifdef LCD // display SETUP (Nokia LCD)
		display.begin();
		displayReset(); //Cleanup the LCD
		display.setContrast(60); // you can change the contrast around to adapt the display for the best viewing!
		display.println("Starting LCD display!");
		display.print("Ver. "); display.println(VERSION);
		display.display(); // show splashscreen
		delay(1000);
	#endif

	#ifdef TFT_ST7735	// display Setup OLED LCD
		display.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab

		// POSITION CURSOR ON TOP LEFT
		displayReset();	
		//display.setContrast(60);
		display.println("Starting TFT display!");
		display.print("Ver:"); display.println(VERSION);
	#endif

	#ifdef TFT_ILI9340 // display Setup OLED LCD

		display.begin();
		//display.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab

		// POSITION CURSOR ON TOP LEFT
		displayReset();
		display.println("Starting TFT display!");
		display.print("Ver:"); display.println(VERSION);
		Serial.println("TFT Display ILI9340 - initialized");
	#endif

	// ### Initialize Radio

	//Radio Setup
	if (!radio.init())
		Serial.println("init failed");
	else { Serial.print("init OK - "); Serial.print(FREQUENCY); Serial.println("mhz"); }
	// Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
	radio.setFrequency(FREQUENCY);

	// ### Initialize Log

	// initialize log variables to start write and read
	mylog.initialize(Data);
	Serial.println("Log data initialized");
	Serial.print("Log next write addr:"); Serial.println(mylog.nextWrite);
	Serial.print("Log next read addr:"); Serial.println(mylog.nextRead);
	Serial.print("Log # records saved:"); Serial.println(mylog.numRecords);

	// load menuscreen on LCD/TFT
	displaymenu(menuPage,false); //Strat menu display (menu page number, screen refresh requirement)

	Serial.println("Setup finished");

	timerLink = millis(); //Initialize Data link loss timeout timer variable
}

void loop()
{

	// Navigate on scrren menu according to buttons

	//Read Buttons states and set menu accordingly
	debouncer1.update();
	debouncer2.update();

	if (debouncer1.fell())  // process button 1 if pressed - Menu navigation
	{
		#ifdef DEBUG
				Serial.println("DEBUG - B1 click");
		#endif
		changeMenu();
		displaymenu(menuPage,true);
		warningLevel = setflag(warningLevel, 0xFF, FLAGRESET); // reset all warning to force re-evaluation
	}
	if (debouncer2.fell())// process button 2 if pressed - Function within Menu
	{
		switch (menuPage)
		{
			case 1: // Navigate 
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
			{
				#ifdef LCD
							level = level + 51;
							if (level > 255) level = 0;
							analogWrite(PIN_LCD_LIGHT, level);
				#endif
				break;
			}
			case 7: // LOG erase
			{
				displaySetCursor(1, 0); display.print(fill(strPRT, SCRCHARS, ' ',true));
				displaySetCursor(2, 0); display.print(fill(strPRT, SCRCHARS, ' ',true));
				displaySetCursor(3, 0); display.print(fill(strPRT, SCRCHARS, ' ',true));
				displaySetCursor(2, 0); sprintf(strPRT, "ERASING LOG..."); display.print(strPRT);

				mylog.eraseData(); //erase LOG memory on B2
				displaymenu(menuPage, true);
				break;
			}

		}
		#ifdef LCD
				display.display();
		#endif
		warningLevel = setflag(warningLevel, 0xFF, FLAGRESET); // reset all warning to force re-evaluation
	}

	// check radio reception and process if data is available
	if (timerLink > millis()) timerLink = millis();
	if (radio.available()) //If some packet was received by the radio, wait for all its contents to come trough
	{
		warningLevel = setflag( warningLevel,WRN_LINK,FLAGRESET); // clear Link flag

		uint8_t len = sizeof(Data);
	
		if (radio.recv((uint8_t *)&Data, &len))
		{
			timerLink = millis(); //Set a counter for data link loss timeout calculation
			rssi = radio.lastRssi(); //RSSI;

				// if GPS fix aquired
			if (Data.fix == 1)
			{
				if (fixinMem == 0) fixposition();
				fixinMem = 1;

				warningLevel = setflag(warningLevel, WRN_FIX, FLAGRESET); 

				kmflag = GPSDist(homelat, homelon,Data.latitudedeg,Data.longitudedeg, &homedist, &homeazim); // Run the distance calculating function, passing the memorized position as arguments				

				// check maximum altitude
				if (Data.altitude > maxalt + homealt)
				{
					maxalt = Data.altitude - homealt;
				}
				// check maximum distance               
				if (kmflag == 0)
				{
					kmflagmem = kmflag;
					if (homedist > maxdist)
					{
						maxdist = homedist;
					}
				}
				else
				{
					if (kmflagmem == 0)
					{
						kmflagmem = 1;
						maxdist = homedist;
					}
					else if (homedist > maxdist)
					{
						maxdist = homedist;
					}
				}

				//check maximum speed
				if (Data.groundspeed*1.852 < 1)
				{
					Data.groundspeed = 0;
				}
				if (Data.groundspeed*1.852 > highspeed)
				{
					highspeed = Data.groundspeed*1.852;
				}
			}
			else
			{
				warningLevel = setflag(warningLevel, WRN_FIX, FLAGSET);
			}
			// save data just received to Log memory
			mylog.saveData(Data);
			displaymenu(menuPage,false); // update menu info (menu page number, screen refresh)
		}
	}
	else // if no data received
	{
		// check if link timeout, ie, no data received for mre than 5 sec
		if (millis() > (timerLink + 5000)) //When LOST for more than 5 sec...
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
	if (millis() < timerWarning) timerWarning = millis(); // if millis() wrap arround reintialize timer
	if (millis() > timerWarning + 2000)
	{
		displaywarning(warningLevel);
		timerWarning = millis();
	}
}

//----------------------------------------------------------------------//
//                             FUNCTIONS                                //
//----------------------------------------------------------------------//


uint8_t setflag(uint8_t flagContainer, uint8_t flag, bool set )
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
	display.setTextSize(CHARSCALE);
	display.setRotation(SCRROTATION);
	display.setTextColor(WHITE,BLACK);
	display.setTextWrap(true);
	display.setCursor(0, 0);
	return;
}

void displaySetCursor(int line, int column)
{
	display.setCursor(column * CHARWIDTH * CHARSCALE, line * CHARHEIGHT*CHARSCALE );
	return;
}

void fixposition()
{
	highspeed = 0;//Reset maximum speed memory
	maxdist = 0; //Reset maximum distane
	maxalt = 0; //Reset maximum altitude
	kmflagmem = 0; //Reset Km flag mem
	homeazim = 0;
	homelat = Data.latitudedeg;//Memorize FIX latitude in DDMM.SS
	homelon = Data.longitudedeg;//Memorize FIX longitude in DDDMM.SS

	/*
	if (Data.lat == 'S') homelat = -homelat;
	if (Data.lon == 'W') homelon = -homelon;
	*/

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
	//Serial.print("Menu:"); Serial.println(menuPage);

	static byte lastmenu;  //remember last menu for paint or refresh

	switch (menuPage)
	{
		case 1: //NAVIGATION STATUS MENU
		{
			if (lastmenu != menuPage || forceRepaint)
				{
					lastmenu = menuPage;
					displayReset();
					displaySetCursor(0, 0); display.print("1 - STATUS");
					displaySetCursor(1, 0); sprintf(strPRT, "SAT:%i", Data.satellites); display.print(strPRT);
					displaySetCursor(2, 0); sprintf(strPRT, "QUAL:%d", Data.fixquality); display.print(strPRT);
					displaySetCursor(3, 0);  sprintf(strPRT, "HDOP:%s", dtostrf(Data.HDOP,5,2,strtmp)); display.print(strPRT);
					displaySetCursor(4, 0); sprintf(strPRT, "RX_RSSI:%d", rssi); display.print(strPRT);
				}
				else // if already in menu, print just info to reduce screen flicker
				{
					displaySetCursor(1, 4); display.print(Data.satellites);
					displaySetCursor(2, 5); display.print(Data.fixquality);
					displaySetCursor(3, 5);  sprintf(strPRT, "%s", dtostrf(Data.HDOP, 5, 2, strtmp)); display.print(strPRT);
					displaySetCursor(4, 8); display.print(rssi);
				}
			break;

		}
		case 2: //INFO MENU
		{
			if (lastmenu != menuPage || forceRepaint)
			{
				lastmenu = menuPage;
				displayReset();
				displaySetCursor(0, 0); display.print("2 - INFO");
				displaySetCursor(1, 0); sprintf(strPRT, "ALT:%s m", dtostrf(Data.altitude - homealt, 4, 0, strtmp)); display.print(strPRT);
				displaySetCursor(2, 0); sprintf(strPRT, "SPD:%s Km/h", dtostrf(Data.groundspeed*1.852, 4, 0, strtmp)); display.print(strPRT);
				displaySetCursor(3, 0); sprintf(strPRT, "AZM:%s Deg", dtostrf(homeazim,4,0,strtmp)); display.print(strPRT);
				displaySetCursor(4, 0); sprintf(strPRT, "DST:%s", dtostrf(homedist,4,0,strtmp)); display.print(strPRT);
				if (kmflag == 0) display.print("m"); else display.print("Km");
				display.setTextSize(CHARSCALE+4);
				displaySetCursor(6, 0); sprintf(strPRT, "%s K/h", dtostrf(highspeed,1,0,strtmp)); display.print(strPRT);
				display.setTextSize(CHARSCALE);
			}
			else
			{
				displaySetCursor(1, 4); sprintf(strPRT, "%s m      ", dtostrf(Data.altitude - homealt, 4, 0, strtmp)); display.print(strPRT);
				displaySetCursor(2, 4); sprintf(strPRT, "%s Km/h      ", dtostrf(Data.groundspeed*1.852, 4, 0, strtmp)); display.print(strPRT);
				displaySetCursor(3, 4); sprintf(strPRT, "%s Deg      ", dtostrf(homeazim, 4, 0, strtmp)); display.print(strPRT);
				displaySetCursor(4, 4); sprintf(strPRT, "%s", dtostrf(homedist,4,0,strtmp)); display.print(strPRT);
				if (kmflag == 0) display.print("m     "); else display.print("Km      "); //Spaces added to allow "m" or "Km" to be erased after large numbers
				display.setTextSize(CHARSCALE + 4);
				displaySetCursor(6, 0); sprintf(strPRT, "%s Km/h", dtostrf(highspeed,1,0,strtmp)); display.print(strPRT);
				display.setTextSize(CHARSCALE);

			}

#ifdef GOOGLEMAPS
			sendToGoogle(Data); 
#endif
			break;
		}

		case 3: //MAXIMUM MENU
		{
			if (lastmenu != menuPage || forceRepaint)
			{
				lastmenu = menuPage;

				displayReset();
				displaySetCursor(0, 0); display.print("3 - MAXIMUM");
				displaySetCursor(1, 0); sprintf(strPRT, "MxSpd:%s", dtostrf(highspeed, 4, 0, strtmp)); display.print(strPRT);
				displaySetCursor(2, 0); sprintf(strPRT, "MxAlt:%4d", maxalt); display.print(strPRT);
				displaySetCursor(3, 0); sprintf(strPRT, "MxDst:%d", maxdist); display.print(strPRT);
				if (kmflag == 0) display.print("m"); else display.print("Km");
				displaySetCursor(4, 0); sprintf(strPRT, "\nGPS Alt:%s",dtostrf( Data.altitude,5,0,strtmp)); display.print(strPRT);

			}
			else
			{

				displaySetCursor(1, 6); sprintf(strPRT, "%s", dtostrf(highspeed, 4, 0, strtmp)); display.print(strPRT);
				displaySetCursor(2, 6); sprintf(strPRT, "%4d", maxalt); display.print(strPRT);
				displaySetCursor(3, 6); sprintf(strPRT, "%d", maxdist); display.print(strPRT);
				if (kmflag == 0) display.print("m"); else display.print("Km");
				displaySetCursor(5, 8); sprintf(strPRT, "%s ", dtostrf(Data.altitude, 5, 0, strtmp)); display.print(strPRT);

			}
			break;
		}

		case 4: //COORDINATES MENU
		{
			if (lastmenu != menuPage || forceRepaint)
			{
				lastmenu = menuPage;

				displayReset();
				displaySetCursor(0, 0); display.print("4 - P. POS");
				displaySetCursor(1, 0); sprintf(strPRT, "LAT:%c %s", Data.lat, dtostrf(Data.latitude, 9, 4, strtmp)); display.print(strPRT);
				displaySetCursor(2, 0); sprintf(strPRT, "LON:%c %s", Data.lon, dtostrf(Data.longitude, 9, 4, strtmp)); display.print(strPRT);
				displaySetCursor(3, 0); sprintf(strPRT, "GOOGLE COORD:"); display.print(strPRT);
				displaySetCursor(4, 0); sprintf(strPRT, "%s", dtostrf(Data.latitudedeg, 10, 6, strtmp)); display.print(strPRT);
				displaySetCursor(5, 0); sprintf(strPRT, "%s", dtostrf(Data.longitudedeg, 10, 6, strtmp)); display.print(strPRT);
			}
			else
			{
				displaySetCursor(1, 4); sprintf(strPRT, "%c %s", Data.lat, dtostrf(Data.latitude, 9, 4, strtmp)); display.print(strPRT);
				displaySetCursor(2, 4); sprintf(strPRT, "%c %s", Data.lon, dtostrf(Data.longitude, 9, 4, strtmp)); display.print(strPRT);
				displaySetCursor(4, 0); sprintf(strPRT, "%s", dtostrf(Data.latitudedeg, 10, 6, strtmp)); display.print(strPRT);
				displaySetCursor(5, 0); sprintf(strPRT, "%s", dtostrf(Data.longitudedeg, 10, 6, strtmp)); display.print(strPRT);
			}
			break;

		}


		case 5: //RECOVERY MENU
		{
			if (lastmenu != menuPage || forceRepaint)
			{
				lastmenu = menuPage;

				displayReset();
				displaySetCursor(0, 0); display.print("5 - RECOVERY");
				displaySetCursor(1, 0); display.print("GOOGLE COORD:");
				displaySetCursor(2, 0); sprintf(strPRT, "%s", dtostrf(Data.latitudedeg, 9, 6, strtmp)); display.print(strPRT);
				displaySetCursor(3, 0); sprintf(strPRT, "%s", dtostrf(Data.longitudedeg, 9, 6, strtmp)); display.print(strPRT);
				displaySetCursor(4, 0); sprintf(strPRT, "AZM:%d", homeazim); display.print(strPRT);
				displaySetCursor(5, 0); sprintf(strPRT, "DIS:%d", homedist); display.print(strPRT);
				if (kmflag == 0) display.print("m"); else display.print("Km");
			}
			else
			{
				displaySetCursor(2, 0); sprintf(strPRT, "%s", dtostrf(Data.latitudedeg, 9, 6, strtmp)); display.print(strPRT);
				displaySetCursor(3, 0); sprintf(strPRT, "%s", dtostrf(Data.longitudedeg, 9, 6, strtmp)); display.print(strPRT);
				displaySetCursor(4, 4); sprintf(strPRT, "%d", homeazim); display.print(strPRT);
				displaySetCursor(5, 4); sprintf(strPRT, "%d", homedist); display.print(strPRT);
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
				displaySetCursor(0, 0); display.print("6 - TOOLS");
				displaySetCursor(1, 0); display.print("Curr. Ver.:");
				displaySetCursor(2, 0); sprintf(strPRT, "%s", VERSION); display.print(strPRT);
			}
			break;
		}

		case 7: //LOG MEN
		{
			if (lastmenu != menuPage || forceRepaint)
			{
				lastmenu = menuPage;

				displayReset();
				displaySetCursor(0, 0); display.print("7 - LOGGER");
				displaySetCursor(1, 0); sprintf(strPRT, "#Records:%lu", mylog.numRecords); display.print(strPRT);
				displaySetCursor(2, 0); sprintf(strPRT, "press B2 to erase!"); display.print(strPRT);
			}
			else
			{
				displaySetCursor(1, 9); sprintf(strPRT, "%lu", mylog.numRecords); display.print(strPRT);
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

		displaySetCursor(SCRLINES , 0);
		display.setTextColor(WHITE, BLACK);
		display.print(fill(strtmp, SCRCHARS, ' ', true));
		displaySetCursor(SCRLINES , 0);

		if (warningcode & WRN_LINK) //link lost
		{
			Serial.print("link Lost");
			display.setTextColor(WHITE, RED);
			display.print("LINK ");

		}
		if (warningcode & WRN_FIX)  // GPS fix lost
		{
			Serial.println("GPS fix Lost");
			display.setTextColor(BLACK, ORANGE);
			display.print(" GPS ");

		}
	

	displaySetCursor(0, 0);
	display.setTextColor(WHITE, BLACK);

#ifdef LCD
	display.display();
#endif

}

void sendToGoogle(Payload strData)
{
	static int oldseconds = 0;

	if (strData.seconds != oldseconds)
	{
		oldseconds = strData.seconds;

		int latint = (int)strData.latitude;
		int latdec = (strData.latitude * 10000) - (latint * 10000);
		int lonint = (int)strData.longitude;
		int londec = (strData.longitude * 10000) - (lonint * 10000);

		// Convert altitude to a string
		char falt[8];
		dtostrf(strData.altitude, 4, 1, falt);

		// Convert speed to a string
		char fspeed[8];
		dtostrf(strData.groundspeed, 4, 2, fspeed);

		// Convert track to a string
		char ftrack[8];
		dtostrf(strData.track, 4, 2, ftrack);

		// Convert HDOP to a string
		char fHDOP[4];
		dtostrf(strData.HDOP, 1, 2, fHDOP);

		// Convert geoidheight to a string
		char fgeoh[8];
		dtostrf(strData.geoidheight, 4, 1, fgeoh);

		//$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
		char *j = gpsstr1;
		j += sprintf(j, "GPGGA,");
		j += sprintf(j, "%.2d%.2d%.2d.000,", strData.hour, strData.minute, strData.seconds, strData.miliseconds); //123519 Fix taken at 12:35 : 19 UTC
		j += sprintf(j, "%.4d.%.4d,%c,", latint, latdec, strData.lat); // 4807.038,N Latitude 48 deg 07.038' N
		j += sprintf(j, "%.5d.%.4d,%c,", lonint, londec, strData.lon); // 4807.038,N Latitude 48 deg 07.038' N
		j += sprintf(j, "1,");                    //   1 Fix quality : 1 - Must always be 1 or we wouldn't be here
		j += sprintf(j, "%.2d,", strData.satellites);        //   08           Number of satellites being tracked
		j += sprintf(j, "%s,", fHDOP);
		j += sprintf(j, "%s,M,", falt);       //   545.4, M      Altitude, Meters, above mean sea level
		j += sprintf(j, "%s,M,,", fgeoh); // Geoid height

		char hexCS1[2];
		sprintf(hexCS1, "%02X", checksum(gpsstr1));
		#ifdef DEBUG
			Serial.print("$"); Serial.print(gpsstr1); Serial.print("*"); Serial.println(hexCS1);
		#endif
		
		//$GPRMC,233913.000,A,3842.9618,N,00916.8614,W,0.50,50.58,180216,,,A*4A
		char *k = gpsstr2;
		k += sprintf(k, "GPRMC,");
		k += sprintf(k, "%.2d%.2d%.2d.000,", strData.hour, strData.minute, strData.seconds, strData.miliseconds);
		k += sprintf(k, "A,"); // A = OK
		k += sprintf(k, "%.4d.%.4d,%c,", latint, latdec, strData.lat); // 4807.038,N Latitude 48 deg 07.038' N
		k += sprintf(k, "%.5d.%.4d,%c,", lonint, londec, strData.lon); // 4807.038,N Latitude 48 deg 07.038' N
		k += sprintf(k, "%s,", fspeed);
		k += sprintf(k, "%s,", ftrack);
		k += sprintf(k, "%.2d%.2d%.2d,,,A", strData.day, strData.month, strData.year);

		char hexCS2[2];
		sprintf(hexCS2, "%02X", checksum(gpsstr2));
		#ifdef DEBUG
			Serial.print("$"); Serial.print(gpsstr2); Serial.print("*"); Serial.println(hexCS2);
		#endif
	}	

	
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

char* fill(char* str, int length, char charcode,bool initialize)
{
	int i =0 ;
	if(!initialize) i = len(str);  // start from were str is already filled
	while (i<length)
	{
		str[i++] = charcode;
	}
	str[i] = NULL;
	return str;
}
