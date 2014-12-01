#include <iostream>
#include <string>
#include <cstring>
#include "TetrisCom.h"
using namespace std;

static const string DEFAULT_SERVER = "127.0.0.1";

/*********************************************************************
 * TetrisCom constructor:  Create a socket connection to the system that
 * communicates with the tetris game.
 */
TetrisCom::TetrisCom()
{
	InitByteCounter();
	bufferSize_ = BUFFER_SIZE; //nbytes + InitByteCounter();
	buffer_ = new char[bufferSize_];

	memset(buffer_,'\0',bufferSize_);
	readSize_ = 1;


#ifdef WIN32
	// Initialize the WINSOCK module.  Since this is the only object
    // in the program that uses WINSOCK, we hide this initialization
    // here.
    //
	WSADATA wsaData;
	WSAStartup(MAKEWORD(2,2), &wsaData);
#endif

	// Create the socket
    //
	robotSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	
	// Ask the user for the name of the robot server,
    // and connect the socket.
    //
	bool socketIsOpen = false;
	while (!socketIsOpen) {
		int rc;
		
		string robotServer;
		cout << "Please enter the name of the tetris server ["
			<< DEFAULT_SERVER << "]: " << flush;
		
		getline(cin, robotServer);

		if (robotServer.empty()) {
			robotServer = DEFAULT_SERVER;
		}

		cout << "You entered "<<robotServer<<endl;

		// connect()
#ifdef WIN32
		SOCKADDR_IN serveraddr;
        serveraddr.sin_family = AF_INET;
        serveraddr.sin_port = htons(PR_NETWORK_PORT_CONTROL);
        serveraddr.sin_addr.s_addr = inet_addr( robotServer.c_str() );
#else
        struct sockaddr_in serveraddr;

        bzero((char *) &serveraddr, sizeof(serveraddr));

        serveraddr.sin_family = AF_INET;

        struct hostent *server;
        server = gethostbyname(robotServer.c_str());
        if (server == NULL) {fprintf(stderr,"ERROR, no such host\n"); exit(0);}
        bcopy((char *)server->h_addr,
              (char *)&serveraddr.sin_addr.s_addr,
              server->h_length);


        serveraddr.sin_port = htons(PR_NETWORK_PORT_CONTROL);
#endif

  
#ifdef WIN32
		rc = connect( robotSocket, (SOCKADDR *)&serveraddr, sizeof(serveraddr) );
#else
        rc = connect( robotSocket, (struct sockaddr *) &serveraddr, sizeof(serveraddr) );
#endif
		if (rc != 0) {
			cerr << "Error: cannot open socket to '" << robotServer
				<< "' port " << PR_NETWORK_PORT_CONTROL << ". Try again."
				<< endl;
			continue;
		}else
		{
			cout << "Connected!" << endl;
		}


		socketIsOpen = true;
	}
	
	// Make the socket blocking
	setSocketBlock( true );
}

void TetrisCom::setSocketBlock( bool fBlock )
{
	// Make the socket non-blocking
	//
	unsigned long nonBlocking;
	if( !fBlock )
		nonBlocking = 1;
	else
		nonBlocking = 0;

#ifdef WIN32
	ioctlsocket(robotSocket, FIONBIO, &nonBlocking);
#else
   int flags = fcntl(robotSocket, F_GETFL, 0);
   if (flags < 0) {
       fprintf(stderr,"setBlocking. something wrong with socket");
       exit(0);
   }
   if (fBlock){
       flags = flags & ~O_NONBLOCK;
   } else {
       flags = flags | O_NONBLOCK;
   }
   flags = fcntl(robotSocket, F_SETFL, flags);
   if (flags < 0) {
       fprintf(stderr,"setBlocking. error setting.");
       exit(0);
   }
#endif
	
	//bufferPtr  = &buffers[0][0];    
	//bufferSize = 0;
}

/*********************************************************************
 * TetrisCom destructor
 */
TetrisCom::~TetrisCom()
{
	delete[] buffer_;

#ifdef WIN32
    closesocket(robotSocket);
    WSACleanup();
#else
    close(robotSocket);
#endif
}

/*********************************************************************
 * sendMessage(): Send a message to the game.
 */
void TetrisCom::sendMessage(const char *ptr)
{
    int bytesLeft = strlen(ptr);
    while (bytesLeft > 0) {
        int sendSize = send(robotSocket, ptr, bytesLeft, 0);
        if (sendSize <= 0) {
            processBrokenSocket();
            return;
        }

        ptr += sendSize;
        bytesLeft -= sendSize;
    }
}
void TetrisCom::sendOK() {
    sendMessage("OK\n");
}

bool TetrisCom::readLine(string& out) {
  out="";
  char c;
  for(;;) {     
    int nread = recv(robotSocket,&c,1,0);
    if(nread<0) return false;
    if(c=='\n') return true;
    out += c;
  }     
}   

/*********************************************************************
 * processBrokenSocket(): Called when the socket connection to the
 * robot is broken.
 */
void TetrisCom::processBrokenSocket()
{
	printf("error: processBrokenSocket()\n");
    return;
}
