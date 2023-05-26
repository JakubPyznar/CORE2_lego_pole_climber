#include "hFramework.h"
#include <stddef.h>
#include <stdio.h>
#include <math.h>
#include <hGPIO.h>
#include <ctype.h>

using namespace hFramework;

int moc_silnikow_odczyt;
int moc_silnikow;

int32_t trzymaj_pozycje1 = hMot1.getEncoderCnt();
int32_t trzymaj_pozycje2 = hMot2.getEncoderCnt();
int wyjscie_PID1 = 0;
int wyjscie_PID2 = 0;
int PID_limit = 700;

bool zazbrojono = false;

void odczyt_CORE2()
{
    while (true)
    {
		if (hExt.pin2.read() && hExt.pin3.read() && hExt.pin4.read() && hExt.pin5.read())
		{
			zazbrojono = false;
		}
		else
		{
			if (hExt.pin2.read()) moc_silnikow_odczyt = 800; else moc_silnikow_odczyt = 0;
        	if (hExt.pin3.read()) moc_silnikow_odczyt = moc_silnikow_odczyt + 400;
       	 	if (hExt.pin4.read()) moc_silnikow_odczyt = moc_silnikow_odczyt + 200;
			if (hExt.pin5.read()) moc_silnikow = moc_silnikow_odczyt; else moc_silnikow = (0-moc_silnikow_odczyt);
			zazbrojono = true;
		}
        hLED2.toggle();
    }
}

void komunikaty_kontrolne()
{
	while (true)
	{
		printf("DI PINY 5|2|3|4: %d|%d|%d|%d\r\nMoc silnikow: %d\r\n Pozycje zapisana i odczytana: %d || %d\r\n\r\n", hExt.pin5.read(), hExt.pin2.read(), hExt.pin3.read(), hExt.pin4.read(), moc_silnikow, trzymaj_pozycje1, hMot1.getEncoderCnt());
		sys.delay(500);
	}
}

void hMain()
{
	sys.setSysLogDev(&DevNull);
	sys.setLogDev(&Serial);				//wszelki logi przekierowane na USB

    hExt.pin2.disableADC(); hExt.pin2.interruptOff(); hExt.pin2.setIn_pd();     //ustawienie pinow GPIO 2..5 do odczytu sygnalow cyforwych od master CORE2
    hExt.pin3.setIn_pd();
    hExt.pin4.setIn_pd();
    hExt.pin5.setIn_pd();

    sys.taskCreate(odczyt_CORE2);
    sys.taskCreate(komunikaty_kontrolne);

    while(true)
    {
		if (zazbrojono == false)
		{
			hMot1.setPower(0);
			hMot2.setPower(0);
		}
		else
		{
			if (moc_silnikow > 0)
			{
				hMot1.setPower(-moc_silnikow);		//ustawienie mocy napedu pionowego nr1
				hMot2.setPower(-moc_silnikow);		//ustawienie mocy napedu pionowego nr2
				trzymaj_pozycje1 = hMot1.getEncoderCnt();
				trzymaj_pozycje2 = hMot2.getEncoderCnt();
			}
			else if (moc_silnikow < 0)
			{
				hMot1.setPower(-moc_silnikow/4);	//ustawienie mocy napedu pionowego nr1
				hMot2.setPower(-moc_silnikow/4);	//ustawienie mocy napedu pionowego nr2
				trzymaj_pozycje1 = hMot1.getEncoderCnt();
				trzymaj_pozycje2 = hMot2.getEncoderCnt();
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
		}
    }
}