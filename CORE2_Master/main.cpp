#include "hFramework.h"
#include <stddef.h>
#include <stdio.h>
#include <DistanceSensor.h>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>

using namespace hFramework;
using namespace hModules;
using namespace std;

char received_BT[1];		//oczyt wiadomosci BT
							//		'S' = stop/reset
							//		'G' = start w gore
							//		'D' = start w dol
							//		'Z' = start zazbrojenie
							//		'R' = rozbrojono

bool pin_state[4];			//przesyl danych do slave CORE2

int moc_radar;				//wartosc zadanej mocy napedu radaru
int radar_encoder_memory = hMot3.getEncoderCnt();		//poczatkowa pozycja radaru
int radar_distance;				//odczyt odleglosci przez sensor ultradzwiekowy
int srednica_rury_cm = 30;
int moc_sprezarka = 0;			//wartosc zadanej mocy napedu sprezarki
int moc_silnikow;				//wartosc zadanej mocy napedow kol robota
int odczytana_moc_silnikow = 250; 	//wartosc odczytanej przez BT mocy
int tryb_pracy = 0;				//zmienna trybu pracy
								//		0 = gotowosc robota
								//		1 = jazda w gore
								//		2 = jazda w dol
								//		3 = zazbrojenie
								//		4 = koniec trasy
								//		5 = tryb automatyczny
int32_t pozycja_auto;
bool kierunek_auto = true;
							
long long zazbrajanie_start;
bool zazbrojono = false;

int32_t trzymaj_pozycje1 = hMot1.getEncoderCnt();
int32_t trzymaj_pozycje2 = hMot2.getEncoderCnt();
int wyjscie_PID1 = 0;
int wyjscie_PID2 = 0;
int PID_limit = 700;

int pompowanie;
long long czas_pompa;

DistanceSensor sens(hSens1);	//deklaracja obiektu sensora odleglosci na porcie hSens1

void odczyt_Bluetooth()
{
	while (true)
	{
		if (hExt.serial.available() != (-1))
		{
			if (hExt.serial.read(received_BT, 1, 10000))
			{
				hLED1.toggle();
				//printf("BT received data: %s\r\n", received_BT);
			}
		}
	}
}

void przesyl_CORE2()
{
	int mnoznik;
	while (true)
	{
		if (zazbrojono)
		{
			mnoznik = abs(moc_silnikow)/200;
			if (moc_silnikow >= 0)  {hExt.pin5.write(true); pin_state[0] = true;} else {hExt.pin5.write(false); pin_state[0] = false;}
			if ((int)(mnoznik/4) == 1) {hExt.pin2.write(true); mnoznik = mnoznik - 4; pin_state[1] = true;} else {hExt.pin2.write(false); pin_state[1] = false;}
			if ((int)(mnoznik/2) == 1) {hExt.pin3.write(true); mnoznik = mnoznik - 2; pin_state[2] = true;} else {hExt.pin3.write(false); pin_state[2] = false;}
			if ((int)(mnoznik/1) == 1) {hExt.pin4.write(true); mnoznik = mnoznik - 1; pin_state[3] = true;} else {hExt.pin4.write(false); pin_state[3] = false;}
		}
		else
		{
			hExt.pin2.write(true); hExt.pin3.write(true); hExt.pin4.write(true); hExt.pin5.write(true);
			pin_state[1] = true; pin_state[2] = true; pin_state[3] = true; pin_state[4] = true;
		}
		hLED2.toggle();
	}
}

void ruch_radar()
{
	while (true)
	{
		if ((tryb_pracy == 1) || (tryb_pracy == 2) || (tryb_pracy == 5))
		{
			if ((hMot3.getEncoderCnt()-radar_encoder_memory) >= 360)
			{
				hMot3.setPower(250);
			}
			else if ((hMot3.getEncoderCnt()-radar_encoder_memory) <= 0)
			{
				hMot3.setPower(-250);
			}
		}
		else hMot3.setPower(0);
		radar_distance = sens.getDistance();
	}
}

void komunikaty_kontrolne()
{
	while (true)
	{
		printf("Tryb pracy: %d\r\nOdczyt BT: %s\r\nMoc silnikow: %d\r\nDO PINY 5|2|3|4: %d|%d|%d|%d\r\nCzujnik: %d\r\nEnkoder radaru: %d\r\n\r\n", tryb_pracy, received_BT, moc_silnikow, pin_state[0], pin_state[1], pin_state[2], pin_state[3], radar_distance, hMot3.getEncoderCnt());
		sys.delay(500);
	}
}

void hMain()
{
	sys.setSysLogDev(&DevNull);
	sys.setLogDev(&Serial);				//wszelki logi przekierowane na USB

	hExt.serial.init(9600, Parity::None, StopBits::One);		//konfiguracja portu szeregowego UART hExt do odbioru danych z modulu HC-06

	hExt.pin2.disableADC(); hExt.pin2.interruptOff(); hExt.pin2.setOut();	//ustawienie pinow 2..5 GPIO do wysylania sygnalow do slave CORE2
	hExt.pin3.setOut();
	hExt.pin4.setOut();
	hExt.pin5.setOut();

	sys.taskCreate(odczyt_Bluetooth);		//task do odczytu komend z modulu BT HC-06
	sys.taskCreate(przesyl_CORE2);			//task do przesylu komend do drugiego sterownika CORE2
	sys.taskCreate(ruch_radar);				//task do obrotu radaru
	sys.taskCreate(komunikaty_kontrolne);	//task do wysylania przez USB komunikatow co 1000ms

	while (true)							//petla glowna programu
	{
		if (((radar_distance > (12/10*srednica_rury_cm/2)) || (radar_distance == -1)) && (tryb_pracy == 1))
		{
			tryb_pracy = 4;
		}

		if (isdigit(received_BT[0]))
		{
			odczytana_moc_silnikow = atoi(&received_BT[0])*200;		//zakres mocy 0,200,..,1000
		}
		
		if ((tryb_pracy == 1) || (tryb_pracy == 2) || (tryb_pracy == 5))
		{
			if ((tryb_pracy == 1) || ((tryb_pracy == 5) && kierunek_auto))
			{
				hMot1.setPower(-moc_silnikow);		//ustawienie mocy napedu pionowego nr1
				hMot2.setPower(-moc_silnikow);		//ustawienie mocy napedu pionowego nr2
			}
			else
			{
				hMot1.setPower(-moc_silnikow/4);	//ustawienie mocy napedu pionowego nr1
				hMot2.setPower(-moc_silnikow/4);	//ustawienie mocy napedu pionowego nr2
			}
			trzymaj_pozycje1 = hMot1.getEncoderCnt();
			trzymaj_pozycje2 = hMot2.getEncoderCnt();
		}
		else if ((tryb_pracy == 0) && (not zazbrojono))
		{
			hMot1.setPower(0);
			hMot2.setPower(0);
		}
		else
		{
			wyjscie_PID1 = -(trzymaj_pozycje1 - hMot1.getEncoderCnt())*10;
			wyjscie_PID2 = -(trzymaj_pozycje2 - hMot2.getEncoderCnt())*10;
			if (wyjscie_PID1 > PID_limit) wyjscie_PID1 = PID_limit; else if (wyjscie_PID1 < -PID_limit) wyjscie_PID1 = -PID_limit;
			if (wyjscie_PID2 > PID_limit) wyjscie_PID2 = PID_limit; else if (wyjscie_PID2 < -PID_limit) wyjscie_PID2 = -PID_limit;
			hMot1.setPower(wyjscie_PID1);
			hMot2.setPower(wyjscie_PID2);
		}

		if (zazbrojono)
		{
			if ((pompowanie == 0) || (pompowanie == 2)) 								{czas_pompa = sys.getRefTime(); pompowanie++;}
			else if ((pompowanie == 1) && ((sys.getRefTime()-czas_pompa) >= 5000)) 		{moc_sprezarka = 1000; pompowanie = 2;}
			else if ((pompowanie == 3  && ((sys.getRefTime()-czas_pompa) >= 2000))) 	{moc_sprezarka = 0; pompowanie = 0;}
		}
		else if (not zazbrojono && (tryb_pracy != 3))									moc_sprezarka = 0;
		hMot4.setPower(moc_sprezarka);

		switch (tryb_pracy)
		{
			case 0:		//GOTOWY
				moc_silnikow = 0;

				if ((received_BT[0] == 'Z') && (not zazbrojono))
				{
					zazbrajanie_start = sys.getRefTime();
					tryb_pracy = 3;
				}
				else if (received_BT[0] == 'G')
				{
					tryb_pracy = 1;
				}
				else if (received_BT[0] == 'D')
				{
					tryb_pracy = 2;
				}
				else if ((received_BT[0] == 'A') && (zazbrojono))
				{
					tryb_pracy = 5;
				}
				else if (received_BT[0] == 'R')
				{
					zazbrojono = false;
				}
				break;

			case 1:		//JAZDA W GORE
				moc_silnikow = odczytana_moc_silnikow;

				if (received_BT[0] == 'S')
				{
					tryb_pracy = 0;
				}
				else if (received_BT[0] == 'D')
				{
					tryb_pracy = 2;
				}
				break;

			case 2:		//JAZDA W DOL
				moc_silnikow = (0-odczytana_moc_silnikow);

				if (received_BT[0] == 'S')
				{
					tryb_pracy = 0;
				}
				else if (received_BT[0] == 'G')
				{
					tryb_pracy = 1;
				}
				break;

			case 3:		//ZAZBRAJANIE
				moc_sprezarka = 1000;
				if ((sys.getRefTime()-zazbrajanie_start) >= 8000)
				{
					zazbrojono = true;
					received_BT[0] = '0';
					moc_sprezarka = 0;
					pompowanie = 0;
					tryb_pracy = 0;
					pozycja_auto = hMot1.getEncoderCnt();
				}
				break;

			case 4:		//PRZESZKODA
				moc_silnikow = 0;

				if (received_BT[0] == 'S')
				{
					tryb_pracy = 0;
				}
				else if (received_BT[0] == 'D')
				{
					tryb_pracy = 2;
				}
				break;
			
			case 5:		//TRYB AUTOMATYCZNY
				if ((radar_distance > (12/10*srednica_rury_cm/2)) || (radar_distance == -1))	kierunek_auto = false;
				else if (hMot1.getEncoderCnt() <= pozycja_auto)									kierunek_auto = true;
				
				if (kierunek_auto) 	moc_silnikow = odczytana_moc_silnikow;
				else 				moc_silnikow = -odczytana_moc_silnikow;

				if (received_BT[0] == 'S')
				{
					tryb_pracy = 0;
				}
				break;
		}
	}
}