/*   Copyright 2018 Guido Zielonkowski

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

/*
 * Das Programm schreibt Datum und Uhrzeit, Temperatur, Spannung und Audiosignal auf SD_Karte.
 * Die LED's signalisieren den Ladezustand des Akkus.
 * Bei weniger als 10,5 Volt wird der Akku mittels Relais abgeschaltet.
 * In Zeile 1 kann ein "Kunde" bzw. eine "Kunden-Nr." eingetragen werden.
 * Die Signalauswertung wird über ein Zeit/Spannungsfenster realisiert.
 * Der ursprüngliche Code stammt aus dem Artikel "Spuren hinterlassen - Datenlogging mit Arduino" von Michael Stal (https://heise.de/-3348205).
 */

#include "Arduino.h"
#include <Wire.h>
#include "RTClib.h"
#include <SPI.h>
#include "SdFat.h"
#include "DallasTemperature.h"
#include "OneWire.h"
#include "TimeLib.h"
#include "DS1307RTC.h"

////////////////////////////////
// DEBUG
///////////////////////////////
#define DEBUG
#ifdef DEBUG
    #define DPRINT(...) Serial.print(__VA_ARGS__)
    #define DPRINTLN(...) Serial.println(__VA_ARGS__)
  #else
    #define DPRINT(...)
    #define DPRINTLN(...)
#endif
  
/////////////////////////////////////////////////////////////////
// Timer / Zeiten in Millisekunden
/////////////////////////////////////////////////////////////////
#define			Time2SDwrite			60000		// Zeit zwischen 2 Schreibvorgängen auf SD-Karte
#define			Time2SDend				300			// Zeit bis Abschaltung nach Entfernung SD-Karte
#define			Time2SDcheck			13000		// Zeit zwischen 2 SD-Karten Prüfungen

#define			Time2Measure			1000		// Zeit zwischen 2 Messungen (Spannung, Temperatur)
#define			Time2Serial				1000		// Zeit bis zur nächsten Ausgabe am seriellen Montior

#define			Time2AutoMute			300	//150		// Zeit bis zur Aktivierung der automatischen Stummschaltung nach Sprechende

#define			Time2MuteBlink			300			// Blink-Puls-Pause-Verhältnis für manuelle Stummschaltung
#define			Time2AlarmBlink			500			// Blink-Puls-Pause-Verhältnis für alarmierende Spannung
#define			Time2KritBlink			200			// Blink-Puls-Pause-Verhältnis für kritische Spannung
#define			Time2Abschalt			10000

#define			Time2Zaehlen			100			// Zeitvorgabe (in Millisekunden) zwischen den Zählerstunden, siehe nächsten Abschnitt

/////////////////////////////////////////////////////////////////
// Zählvorgaben für Signalauswertung
/////////////////////////////////////////////////////////////////
#define			ZAEHLER_AN				20  //30				// Diese Variable sorgt für Einschaltverzögerung/sicherer Signalerkennung bei anliegendem Signal.
#define			ZAEHLER_AUS				250 //200				// Rechenbeispiel: Zahler_AN * Time2Zaehlen (z.B. 30 * 100ms = 3000ms = 3s)

/////////////////////////////////////////////////////////////////
// Anschlusspins
/////////////////////////////////////////////////////////////////
#define			Spannungspin			A1	// Hauptspannungsmessung
#define			LEDgn					5
#define			LEDge					6
#define			LEDrt					7
#define			Relais					8	// Einschaltrelais Hauptspannung
#define			monoflopPin				A3	// Port, an den der Monoflop-Ausgang angeschlossen wird
#define			TemperaturPin			2	// DS1820 Temperatursensor
#define			MutePin					9	// Betätigungstaste für Stummschaltung, Eingang
#define			SwitchMutePin			3	// Relais für Stummschaltung, Ausgang

/////////////////////////////////////////////////////////////////
// Definition der Wertebereiche
/////////////////////////////////////////////////////////////////

// Innerhalb den Wertebereiche stumm schalten, oberhalb und unterhalb Toleranz laut schalten
/*
 * Festlegung der Mikrofonwerte:
 * Mittenwert:	28
 * Toleranz:	+/- 100 = (Wertbereich) +128 / -72 für AUS! (innerhalb)
 * Min-/Max-Toleranz: +/- 350 = (Wertbereich) +378 / -322 für EIN! (außerhalb)
 */
#define			SIGNAL_MITTE			28L // Ohne Wertigkeit (V, A o.ä.), da keine Umrechnung möglich oder nötig (L stehen lassen, da Vorzeichen-behaftete Berechnung)
#define			TOLERANZ_AUS			180L//200L // Aus-Wertbereich, bei dem jeweils von der Mitte aus gesehen, ausgeschaltet werden soll.
#define			TOLERANZ_EIN			350L//500L // Ein-Wertbereich, bei dem jeweils von der Mitte aus gesehen, eingeschaltet werden soll.

#define			TEMP_AUS_ZAEHLER		100
#define			TEMP_AN_ZAEHLER			10

//////////////////////////////////////////////////////////////////////////////////////////
/*
 * Mehere Spannungsbereiche sind definiert:
 *
 * Volt
 * 14,5V	^
 * ....V	|
 * 12,9V	o	^
 * 12,6V		o	^
 * 12,4V			o	^
 * 11,8V				o	^
 * ....V					|
 * 0,50V					o	^
 * 0,00V						o
 ********************************* Abschnitte
 *			A	B	C	D	E	F
 */

#define			VOLT_OK(Spannung)			(Spannung <= 14.5 && Spannung > 12.7) 	// A
#define			VOLT_WARN(Spannung)			(Spannung <= 12.7 && Spannung > 12.2) 	// B
#define			VOLT_ALARM(Spannung)		(Spannung <= 12.2 && Spannung > 11.7) 	// C
#define			VOLT_KRIT(Spannung)			(Spannung <= 11.7 && Spannung > 11.2) 	// D
#define			VOLT_AUS(Spannung)			(Spannung <= 11.2 && Spannung > 0.5)	// E
#define			VOLT_TEST(Spannung)			(Spannung <= 0.5 && Spannung > 0.0)		// F

//////////////////////////////////////////////////////////////////////////////////////////

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// Diesen Abschnitt NICHT ändern!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

/////////////////////////////////////////////////////////////////
// DEKLARATIONEN & Definitionen
/////////////////////////////////////////////////////////////////
#define			LOGDATEI 				"Log_"
#define			PRAEFIXLAENGE sizeof(LOGDATEI) - 1
#define			TemperaturZeichen		'C'
#define			AnzahlDezimalStellen 	4

#define			SEP						F(";") // Trennzeichen für Dateiausgabe
#define			error(msg) sd.errorHalt(F(msg)) // Fehlermeldungen im Flash ablegen.

#define			chipSelect				SS // Chip Select auf Arduino Board

#define			AN						1
#define			AUS						0
#define			UNVERAENDERT			2
#define			OK						1
#define			NOK						0
#define			NEIN					0
#define			JA						1
#define			MAN						2
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

const char WochenTage[7][12] =
{ "Sonntag", "Montag", "Dienstag", "Mittwoch", "Donnerstag", "Freitag", "Samstag" };

char dateiName[13] = LOGDATEI "0000.csv"; // Z.B. Log_0004.csv
bool SDcheck = 1;
DateTime jetzt;

typedef enum e_millis_num {_SDwrite, _SDcheck, _SDend, _MuteBlink, _Check, _StummBlink, _AutoStumm, _AlarmBlink, _KritBlink, _Serial, _Zaehlen, _Abschalt, reset, _NO_States} ms;
uint16_t const Set[_NO_States] = {Time2SDwrite, Time2SDcheck, Time2SDend, Time2MuteBlink, Time2Measure, Time2MuteBlink, Time2AutoMute, Time2AlarmBlink, Time2KritBlink, Time2Serial, Time2Zaehlen, Time2Abschalt};
uint32_t T[_NO_States] = {0};

SdFat sd; 		// Zugriff auf Dateisystem der SD Karte
SdFile datei; 	// Log-Datei
RTC_DS1307 rtc;	// Zugriff auf RTC
OneWire oneWire(TemperaturPin);
DallasTemperature OneWireSensor(&oneWire);


// Audiosignalauswertung
bool Signal_EIN(int32_t SignalSpg)
{
	return (SignalSpg < (SIGNAL_MITTE - TOLERANZ_EIN) || SignalSpg > (SIGNAL_MITTE + TOLERANZ_EIN));
}

bool Signal_AUS(int32_t SignalSpg)
{
	return (SignalSpg > (SIGNAL_MITTE - TOLERANZ_AUS)  && SignalSpg < (SIGNAL_MITTE + TOLERANZ_AUS));
}

void dateTime(uint16_t* date, uint16_t* time) // Callback-Funktion für Datum/Uhrzeit der Datei
{
	DateTime now = rtc.now();
	*date = FAT_DATE(now.year(), now.month(), now.day());
	*time = FAT_TIME(now.hour(), now.minute(), now.second());
}

bool timing(const ms Timer, const ms tmp = reset)
{
	static ms RESET = reset;
	if(Timer == reset)
	{
		if (tmp != reset)	T[tmp] = millis();
		else				T[RESET] = millis();
		return 0;
	}
	RESET = Timer;
	if ((millis() - T[Timer]) > Set[Timer]) return 1;
	else return 0;
}

void initRTC()	// Echtzeituhr initialisieren
{
	if (!rtc.begin())
	{ // ist eine Uhr angeschlossen?
		Serial.println("Echtzeituhr fehlt");
		while (1); // Fehlerschleife
	}
	if (!rtc.isrunning())
	{ // Uhr schon gesetzt?
		Serial.println("RTC bisher noch nicht gesetzt!");
		// => Initialisiere Zeit und Datum auf Compile-Zeit
		rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
	}
}

/*  code to process time sync messages from the serial port   */
#define TIME_HEADER  'T'   // Header tag for serial time sync message

unsigned long processSyncMessage(void) {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
     return pctime;
     if( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
       pctime = 0L; // return 0 to indicate that the time is not valid
     }
  }
  return pctime;
}

void initSDCardReader(void)		// Kartenleser initialisieren
{
	// SD Card mit SPI_HALF_SPEED initialisieren, um Fehler bei Breadboardnutzung zu vermeiden.  Sonst => SPI_FULL_SPEED
	if (!sd.begin(chipSelect, SPI_HALF_SPEED))
		{
			digitalWrite(LEDgn, LOW);
			sd.initErrorHalt();  // Zugriff auf SD?
		}
	if (PRAEFIXLAENGE > 8)
		error("Dateipr\344fix zu lang"); // Dateiformat 8.3
	Serial.println("SD Prefix");
	// Standarddateiname LOGDATEI  + laufende Nummer, z.B. TMP3603.csv
	// Sobald alle Suffixe 00..99 verbraucht sind, geht es von vorne los: round robin

	  while (sd.exists(dateiName)) {
	    if (dateiName[PRAEFIXLAENGE + 3] != '9') {
	     dateiName[PRAEFIXLAENGE + 3]++;
	    }
	    else if (dateiName[PRAEFIXLAENGE + 2] != '9') {
	      dateiName[PRAEFIXLAENGE + 3] = '0';
	      dateiName[PRAEFIXLAENGE + 2]++;
	    }
	    else if (dateiName[PRAEFIXLAENGE + 1] != '9') {
	      dateiName[PRAEFIXLAENGE + 2] = '0';
	      dateiName[PRAEFIXLAENGE + 1]++;
	    }
	    else if (dateiName[PRAEFIXLAENGE] != '9') {
	      dateiName[PRAEFIXLAENGE + 1] = '0';
	      dateiName[PRAEFIXLAENGE]++;
	    }
	    else {
	      error("Kann Datei nicht erzeugen");
	    }
	  }

	SdFile::dateTimeCallback(dateTime);

	if (!datei.open(dateiName, O_CREAT | O_WRITE | O_EXCL))
		{
			digitalWrite(LEDgn, LOW);
			error("Datei \366ffnen misslungen!"); // Jetzt öffnen:
		}
	else
	{
		Serial.print(F("Logging auf: "));
		Serial.println(dateiName);
	}
	schreibeHeader(); // Header schreiben
}

class CommaForDot : public Print	// Ersetze Punkt durch Komma
{
public:
    CommaForDot(Print &downstream) : downstream(downstream) {}
    virtual size_t write(uint8_t c)
    {
    	return downstream.write(c=='.' ? ',' : c);
    }
private:
    Print &downstream;
};

void WertAusgeben(const String PreCursor = "",const float Wert = 0.0,const String PostCursor = "")
{
	CommaForDot filterSerial(Serial);
	Serial.print(PreCursor);
	filterSerial.print(Wert);
	Serial.println(PostCursor);
}

void ZeitAusgeben(const DateTime *jetzt)
{
	Serial.print(jetzt->year(), DEC);
	Serial.print(jetzt->month() < 10 ? "/0" : "/");
	Serial.print(jetzt->month(), DEC);
	Serial.print(jetzt->day() < 10 ? "/0" : "/");
	Serial.print(jetzt->day(), DEC);
	Serial.print(" (");
	Serial.print(WochenTage[jetzt->dayOfTheWeek()]);
	Serial.print(jetzt->hour() < 10 ? ") 0" : ") ");
	Serial.print(jetzt->hour(), DEC);
	Serial.print(jetzt->minute() < 10 ? ":0" : ":");
	Serial.print(jetzt->minute(), DEC);
	Serial.print(jetzt->second() < 10 ? ":0" : ":");
	Serial.println(jetzt->second(), DEC);
}

void schreibeHeader(void)
{
	datei.println(F("Logdatei f\374r Kunde"));
	datei.print(F("Datum"));
	datei.print(SEP);
	datei.print(F("Uhrzeit"));
	datei.print(SEP);
	datei.print(F("Temperatur"));
	datei.print(SEP);
	datei.print(F("Spannung"));
	datei.print(SEP);
	datei.print(F("Signal"));
	datei.print(SEP);
	datei.print(F("Stumm"));
	datei.print(SEP);
	datei.println(F("Abschaltung"));
	if (!datei.sync() || datei.getWriteError())
	{
		error("Schreibfehler!");
		SDcheck = 0;
	}
}

void schreibeMessung(const float &Temperatur,const DateTime &jetzt,const float &Spannung,const bool &Signal, const uint8_t &Stummschaltung)
{
	if (datei.isOpen())
	{
		CommaForDot filterDatei(datei);
		datei.print(jetzt.day());
		datei.print(F("."));
		datei.print(jetzt.month());
		datei.print(F("."));
		datei.print(jetzt.year());
		datei.print(SEP);
		datei.print(jetzt.hour());
		datei.print(jetzt.minute() < 10 ? ":0" : ":");
		datei.print(jetzt.minute());
		datei.print(jetzt.second() < 10 ? ":0" : ":");
		datei.print(jetzt.second());
		datei.print(SEP);
		filterDatei.print(Temperatur);
		datei.print(SEP);
		filterDatei.print(Spannung);
		datei.print(SEP);
		datei.print(Signal==1?"Ja":"");
		datei.print(SEP);

		switch (Stummschaltung)
		{
			case NEIN: datei.print(""); break;
			case JA: datei.print("auto"); break;
			case MAN: datei.print("Manuell"); break;
		}

//		datei.print(Stummschaltung==0?"1":"");
		datei.print(SEP);
		if (VOLT_AUS(Spannung)) datei.println("Ja"); //Unterspannungsabschaltung
		else datei.println("");
		if (!datei.sync() || datei.getWriteError())
		{
			Serial.println("Schreibfehler!"); // Dateisync, um Datenverlust zu vermeiden:
			SDcheck = 0;
		}
	}
	else
		{
		Serial.println("Datei \366ffnen misslungen!");
		SDcheck = 0;
		}
}

void SerialMonitor(const bool &Signal,const float &Spannungswert,const float &Temperatur,const uint32_t &SignalSpg, const bool &stumm)
{
	if (timing(_Serial)) // jede x Sekunden Daten auf seriellem Port ausgeben
	{
		ZeitAusgeben(&jetzt); // Datum/Uhrzeit schreiben am seriellen Monitor
		WertAusgeben("Spannung: ", Spannungswert, " V="); //Endgültigen Spannungswert im seriellen Monitor anzeigen
		WertAusgeben("Temperatur: ",Temperatur," \260C");
		Serial.print(Signal == AN ? "Signal" : "- -- -");
		Serial.println(stumm == AN ? " Auto-Stumm" : " LAUT");
		Serial.print("Signalstärke: ");
		Serial.println(SignalSpg);
		timing(reset);
	}
}

void Blinken(uint8_t LED)
{
	static bool tmp[16] = {0};
	digitalWrite(LED, tmp[LED] ^= 1);
}

uint8_t AutoMute(const uint32_t &Signal, const bool &manual_mute)
{
	static bool tmp = AUS;
	static uint8_t last = AN;
	static uint16_t temp2off = 0;
	static uint16_t temp2on = 0;

// Manual-Mute
	if (manual_mute)
	{
		digitalWrite(MutePin, HIGH);
		last = MAN;
		return (MAN);
	}
// Stummschaltung aufheben
	else if (Signal_EIN(Signal))
	{
		if (!tmp)
		{
			if (temp2on < TEMP_AN_ZAEHLER) temp2on++;
			else if (temp2on >= TEMP_AN_ZAEHLER)
			{
				digitalWrite(MutePin, LOW);
				tmp = AN;
				last = AUS;
				temp2on = 0;
				return AUS;
			}
		}

	}
// Bereich zwischen Ein- und Ausschaltung "ausblenden" mittels Timer-Reset und Merkervariable

// stummschalten, wenn Zeit abgelaufen
	else if (Signal_AUS(Signal))
	{
		if (timing(_AutoStumm) && !tmp)
		{
			digitalWrite(MutePin, HIGH);
			tmp = AN;
			last = AN;
			return AN;
		}
		else if (temp2off >= TEMP_AUS_ZAEHLER)
		{
			timing(reset, _AutoStumm);
			tmp = AUS;
			temp2off = 0;
		}
		else if (tmp)
		{
			temp2off++;
		}
	}
	return last;
}

// Auswertung des AutoMute Signals zur Ermittlung des Langzeitzustands
uint8_t Auswertung(const bool &Spannung,const uint8_t &Mute_Signal)
{
	static uint8_t chunks = 0;
	static uint8_t Status = AUS;
	static uint8_t upcount = 0;

	if (!Spannung) return AUS;

	if (timing(_Zaehlen))
	{
		timing(reset);

		if (Status == Mute_Signal)
		{
			if ((Status == AUS || Status == MAN) && chunks >= ZAEHLER_AN)
			{
				chunks = 0;
				Status = AN;
			}
			else if (Status == AN && chunks >= ZAEHLER_AUS)
			{
				chunks = 0;
				Status = AUS;
			}
			else chunks++;
		}
		else if (chunks > 0)
		{
			if (upcount >= 10)
			{
				if (Status == AUS || Status == MAN)
				{
					chunks--;
					upcount = 0;
				}
				else if (Status == AN)
				{
					chunks = 0;
					upcount = 0;
				}
			}
			else upcount++;
		}
	}
	return Status;
}

void setup()
{
	Serial.begin(115200); // Serielle Kommunikation an
  DPRINTLN("[0/7] Serial begin");
	pinMode(LEDgn, OUTPUT);
	digitalWrite(LEDgn, HIGH);
	pinMode(LEDge, OUTPUT);
	pinMode(LEDrt, OUTPUT);
	pinMode(Relais, OUTPUT);
	digitalWrite(Relais, LOW);
	pinMode(monoflopPin, INPUT_PULLUP);
	pinMode(MutePin, OUTPUT);
	pinMode(SwitchMutePin, INPUT_PULLUP);
 
  DPRINTLN("[1/7] Initializing RTC");

	initRTC();          // Echtzeituhr initialisieren
	setSyncProvider(RTC.get);   // the function to get the time from the RTC

	if (timeStatus() != timeSet) Serial.println("Unable to sync with the RTC");
	else Serial.println("RTC has set the system time");

  DPRINTLN("[2/7] RTC initialized. Initializing SD-Card");

	initSDCardReader(); // SD Card initialisieren

  DPRINTLN("[3/7] Starting temperature sensor");
	
	OneWireSensor.begin();		// Temperatursensor initialisieren

  DPRINTLN("[4/7] Requesting temperatures...");

	OneWireSensor.requestTemperatures();
	delay(600);
	OneWireSensor.requestTemperatures();
  
  DPRINTLN("[5/7] Request sent. Activating Relay.");
	
	OneWireSensor.setWaitForConversion(false);
	digitalWrite(Relais, HIGH);

  DPRINTLN("[6/7] Relay active. Syncing time from RTC.");

	jetzt = rtc.now();

  DPRINTLN("[7/7] Setup complete, entering loop.");
}

void loop()
{
	static uint8_t SignalZustand = AUS; // Variable, die den Zustand des Monoflop speichert
	static uint8_t alterZustand = AN;
	static uint8_t autoStumm = AUS;

	static float Spannung = 13.0;
	static float Temperatur = OneWireSensor.getTempCByIndex(0);

	static int32_t Signal = SIGNAL_MITTE;

	bool stumm = !digitalRead(SwitchMutePin);  //Low-active Schalter, deshalb als Auswertung invertiert (stumm == AN/AUS)

	if (Serial.available())
	{
		time_t t = processSyncMessage();
		if (t != 0)
		{
			RTC.set(t); // setze RTC und Systemzeit auf empfangenen Wert
			setTime(t);
		}
	}
// alle x-Sekunden auf SD-Karte prüfen

	if (timing(_SDcheck))
	{
		SDcheck = sd.exists(dateiName) > 0?1:0;
		timing(reset);
	}

	if (timing(_Check))
	{
		Spannung = ((int(analogRead(Spannungspin) / 4.085) % 1000) / 10.0); //Spannungswert am analogen Eingang auslesen und mathematisch umwandeln um den Spannungswert in Volt zu erhalten
		jetzt = rtc.now(); // Datum & Zeit holen
		OneWireSensor.requestTemperatures(); // Temperaturmesswert auslesen
		Temperatur = OneWireSensor.getTempCByIndex(0);
		timing(reset);
	}

	SerialMonitor(SignalZustand, Spannung, Temperatur, Signal, autoStumm);

// Stummschaltung(Mute) mit grüner LED blinkend!!!
	if (stumm && timing(_MuteBlink))
	{
		Blinken(LEDgn);
		timing(reset);
	}
	else if (!stumm)
	{
		if (VOLT_OK(Spannung))	digitalWrite(LEDgn, HIGH);
		else					digitalWrite(LEDgn, LOW);
	}

// Gelbe LED leuchten/blinken/schnell blinken bei Spannungsabfall
	if (VOLT_OK(Spannung) || VOLT_TEST(Spannung))	digitalWrite(LEDge, LOW);
	else if (VOLT_WARN(Spannung))					digitalWrite(LEDge, HIGH);
	else if (VOLT_ALARM(Spannung) && timing(_AlarmBlink))
	{
		Blinken(LEDge);
		timing(reset);
	}
	else if (VOLT_KRIT(Spannung) && timing(_KritBlink))
	{
		Blinken(LEDge);
		timing(reset);
	}

// schaltet Relais frei, wenn SD-Karte vorhanden und Spannung ausreichend
	if (!(VOLT_AUS(Spannung) || VOLT_TEST(Spannung)) && SDcheck == 1) digitalWrite(Relais, HIGH);
	else if ((VOLT_AUS(Spannung) || (Temperatur >= 60.0)) && timing(_Abschalt))
	{
		schreibeMessung(Temperatur, jetzt, Spannung, SignalZustand, autoStumm);
		datei.close();
		digitalWrite(Relais, LOW);
		timing(reset);
	}
	else if (!SDcheck) digitalWrite(Relais, LOW);

	Signal = analogRead(monoflopPin);

	autoStumm = AutoMute(Signal, stumm);

	// Es wird in Zeitabschnitten ausgewertet. 1 Abschnitt entspricht 1 Sekunde
	SignalZustand = Auswertung(!(VOLT_AUS(Spannung)||VOLT_TEST(Spannung)), autoStumm);

	// Daten schreiben, wenn a) SD-Karte vorhanden UND b1)Zeit abgelaufen ODER b2)Änderung im Signal
	if (SDcheck && !(VOLT_AUS(Spannung) || VOLT_TEST(Spannung)) && (timing(_SDwrite) || SignalZustand != alterZustand ))
	{
		schreibeMessung(Temperatur, jetzt, Spannung, SignalZustand, autoStumm);
		timing(reset);
		alterZustand = SignalZustand;
	}
	else if (!SDcheck && timing(_SDend))
	{
		Blinken(LEDgn);
		timing(reset);
		if (!sd.begin(chipSelect, SPI_HALF_SPEED)) Serial.println("SD-Karte wurde entfernt");
		else if (!datei.isOpen())
			{
				datei.open(dateiName, O_RDWR|O_AT_END);
				timing(_SDwrite);
			}
	}
}
