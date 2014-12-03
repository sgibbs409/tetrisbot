
#ifndef _magnet_com_h
#define _magnet_com_h

// Empty magnet functions for compiling when not on Windows

typedef int HANDLE;
HANDLE magnetInit(char*) {return 0;}
void magnetTest (HANDLE hSerial) {}
void magnetOn (HANDLE hSerial) {}
void magnetOff (HANDLE hSerial) {}
void magnetClose(HANDLE hSerial) {}
void _sleep(int sec) {}


#endif
