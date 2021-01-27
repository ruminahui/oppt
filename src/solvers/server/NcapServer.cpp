#include "NcapServer.hpp"








int main(int argc, char const* argv[])
{    
	const std::string ipc_path = "ipc:///home/jimy/Desktop/ncap_ipc/1";
	// Planning time in ms
	FloatType planningTime = 300; 
    // Create an NCAP Server object
    NcapServer server{planningTime};

    // Attemtp to set up problem environment within server
    if(!server.setupOpptEnvironment(argc, argv)){
    	return -1; // FAILED
    }


    // Environment set up was successful, start up server
    server.bindServer(ipc_path);

    
    while(true){
    	// Keep serving
    	server.parseMessage();
    }

}



