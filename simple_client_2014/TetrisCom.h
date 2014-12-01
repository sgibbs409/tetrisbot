#ifndef _TETRIS_COM_H_
#define _TETRIS_COM_H_

#define BUFFER_SIZE (PR_NETWORK_BUFFER_SIZE + 4)
#include "unitMsg.h"
#include "Robot/cs225.h"
#include <string>

class TetrisCom {
public:
	TetrisCom();
	~TetrisCom();
        bool readLine(std::string& out);
        void sendOK();
	void setSocketBlock( bool fBlock );

private:
#ifdef WIN32
    SOCKET robotSocket;
#else
    int robotSocket;
#endif
    void sendMessage(const char* str);
    void processBrokenSocket();
	int InitByteCounter();
	
	char *buffer_;
	int bufferSize_;
	int byteCounter_;

	short size_;
	int readSize_;

};

inline int TetrisCom::InitByteCounter()
{
  byteCounter_ = 4; // 2*sizeof(short);
  return byteCounter_;
}

#endif // _TETRIS_COM_H_
