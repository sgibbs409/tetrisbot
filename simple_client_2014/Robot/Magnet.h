#ifndef _magnet_com_h
#define _magnet_com_h


HANDLE magnetInit(char*);
void magnetTest (HANDLE hSerial);
void magnetOn (HANDLE hSerial);
void magnetOff (HANDLE hSerial);
void magnetClose(HANDLE hSerial);


#endif
