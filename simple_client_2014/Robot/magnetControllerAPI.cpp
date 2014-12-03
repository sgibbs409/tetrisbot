// PumaClient.cpp : Defines the entry point for the console application.
//
#pragma once

//#include "stdafx.h"
#include <Windows.h>
#include "Magnet.h"


#include <iostream>
#include <tchar.h>
using namespace std;



/* Open Communication Link to COM Port
 * portName: COM port string name.  Example openComs("COM6");
 */

HANDLE magnetInit(char* portName)
{
	//Open USB port COM3
	bool result = false;
	HANDLE hSerial;
 
	hSerial = CreateFile(portName, GENERIC_READ | GENERIC_WRITE, 0, 0,OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
	if(hSerial == INVALID_HANDLE_VALUE)
	{
		result = false;
	}
	else
	{
		result = true;
	}

	DCB dcbSerialParams = {0};
	dcbSerialParams.DCBlength=sizeof(dcbSerialParams);

	if (!GetCommState(hSerial, &dcbSerialParams))
	{
		result = false;
		CloseHandle(hSerial);
	}
	
	dcbSerialParams.BaudRate = CBR_9600;
	dcbSerialParams.ByteSize = 8;
	dcbSerialParams.StopBits = ONESTOPBIT;
	dcbSerialParams.Parity = NOPARITY;

	if(!SetCommState(hSerial, &dcbSerialParams))
	 {
		result = false;
		CloseHandle(hSerial);
	 }

	COMMTIMEOUTS timeouts = {0};
	 timeouts.ReadIntervalTimeout = 10;
	 timeouts.ReadTotalTimeoutConstant = 10;
	 timeouts.ReadTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = 10;
	 timeouts.WriteTotalTimeoutMultiplier = 10;
	 if(!SetCommTimeouts(hSerial, &timeouts))
	 {
		result = false;
		CloseHandle(hSerial);
	 }

	if (!result) cout << "Error initializing serial communication\n";

	 return hSerial;
}

void magnetOn (HANDLE hSerial)
{
	unsigned char sendBuff[4] = {0};
	DWORD dwBytesRead = 0;
	
	sendBuff[0] = 'a';

	if(!WriteFile(hSerial, &sendBuff, 1, &dwBytesRead, NULL))
	{
		cout << "Serial communication error!\n";
	}
		
	return;
}

void magnetOff (HANDLE hSerial)
{
	unsigned char sendBuff[4] = {0};
	DWORD dwBytesRead = 0;
	
	sendBuff[0] = 'b';

	if(!WriteFile(hSerial, &sendBuff, 1, &dwBytesRead, NULL))
	{
		cout << "Serial communication error!\n";
	}
		
	return;
}

void magnetClose(HANDLE hSerial)
{
	CloseHandle(hSerial);
	return;
}


void magnetTest(HANDLE hSerial)
{
	unsigned char sendBuff[4] = {0};
	DWORD dwBytesRead = 0;
	while(1)
	 {
		 /* Turn on Magnet */
		 Sleep(5000);
		
		 sendBuff[0] = 'a';

		 
		 if(!WriteFile(hSerial, &sendBuff, 1, &dwBytesRead, NULL))
		 {
			cout << "Serial communication error!\n";
		 }
		 else
		 {
			 cout << "Magnet ON\n";
		 }


		/* Turn off magnet */
		  Sleep(5000);
		// sending 4 bytes
		 sendBuff[0] = 'b';

		 
		 if(!WriteFile(hSerial, &sendBuff, 1, &dwBytesRead, NULL))
		 {
			cout << "Serial communication error!\n";
		 }
		  else
		 {
			 cout << "Magnet OFF\n";
		 }

	 }
	return;	
}

