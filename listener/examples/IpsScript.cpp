/* 
    Example of two different ways to process received OSC messages using oscpack.
    Receives the messages from the SimpleSend.cpp example.
*/

#include <iostream>
#include <fstream>
#include <cstring>

#if defined(__BORLANDC__) // workaround for BCB4 release build intrinsics bug
namespace std {
using ::__strcmp__;  // avoid error: E2316 '__strcmp__' is not a member of 'std'.
}
#endif

#include "osc/OscReceivedElements.h"
#include "osc/OscPacketListener.h"
#include "ip/UdpSocket.h"
#include "osc/OscOutboundPacketStream.h"
#include <cstdlib>
#include <string>
#include <sstream>

#define RECEIVING_PORT 7000
#define SENDING_PORT 8000
#define ADDRESS "127.0.0.1"
#define OUTPUT_BUFFER_SIZE 1024

class ExamplePacketListener : public osc::OscPacketListener {
protected:

    virtual void ProcessMessage( const osc::ReceivedMessage& m, 
				const IpEndpointName& remoteEndpoint )
    {
        (void) remoteEndpoint; // suppress unused parameter warning

        try {
            // example of parsing single messages. osc::OsckPacketListener

            //std::cout << "m.AddressPattern():" << m.AddressPattern() << std::endl;

            // handles the bundle traversal.
            
            //if ( std::strcmp( m.AddressPattern(), "/b/IPS.LIDAR_START.Value" ) == 0 ) {
            if ( std::strcmp( m.AddressPattern(), "LIDAR_START" ) == 0 ) {
                osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
                float message;
                args >> message >> osc::EndMessage;
                
                if (message == 1) {
                    std::cout << "starting visualizer..." << std::endl;
                     std::system("~/quanergy_client/build/visualizer --host 192.168.88.12 &");
                } else if (message == 0) {
                    std::system("killall visualizer");
                } else {

                }
            } else if ( std::strcmp( m.AddressPattern(), "LIDAR_ACTIVATE" ) == 0 ) {
                osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
                float value;
                args >> value >> osc::EndMessage;

                std::ofstream file;
                file.open("../../setBaseline.json");
                file << "{ \"LIDAR_ACTIVATE\":\"";
                file << value;
                file << "\"}";
                file.close();

            } else if ( std::strcmp( m.AddressPattern(), "MARGIN_HORIZONTAL" ) == 0 ) {
                osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
                float value;
                args >> value >> osc::EndMessage;

                //std::cout << "MARGIN_HORZONTAL " << value << std::endl;

                std::ofstream file;
                file.open("../../setBaseline.json");
                file << "{ \"MARGIN_HORZONTAL\":\"";
                file << value;
                file << "\"}";
                file.close();
            } else if ( std::strcmp( m.AddressPattern(), "MARGIN_VERTICAL" ) == 0 ) {                osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
                float value;
                args >> value >> osc::EndMessage;

                //std::cout << "MARGIN_VERTICAL " << value << std::endl;

                std::ofstream file;
                file.open("../../setBaseline.json");
                file << "{ \"MARGIN_VERTICAL\":\"";
                file << value;
                file << "\"}";
                file.close();
            } else if ( std::strcmp( m.AddressPattern(), "DISTANCE_THRESHOLD" ) == 0 ) {                osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
                float value;
                args >> value >> osc::EndMessage;

                std::cout << "DISTANCE_THRESHOLD " << value << std::endl;

                std::ofstream file;
                file.open("../../setBaseline.json");
                file << "{ \"DISTANCE_THRESHOLD\":\"";
                file << value;
                file << "\"}";
                file.close();
            }

        } catch( osc::Exception& e ) {
            // any parsing errors such as unexpected argument types, or 
            // missing arguments get thrown as exceptions.
            std::cout << "error while parsing message: "
                << m.AddressPattern() << ": " << e.what() << "\n";
        }
    }
};

int main(int argc, char* argv[])
{
    (void) argc; // suppress unused parameter warnings
    (void) argv; // suppress unused parameter warnings

    // Send message to start communication via osc
    UdpTransmitSocket transmitSocket( IpEndpointName( ADDRESS, SENDING_PORT ) );
    
    char buffer[OUTPUT_BUFFER_SIZE];
    osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
    
    /*p << osc::BeginBundleImmediate
        << osc::BeginMessage("/b/IPS.LIDAR_START.Value") << 0
        << osc::EndMessage
        << osc::BeginMessage("/b/IPS.LIDAR_ACTIVATE.Value") << 0
        << osc::EndMessage
        << osc::BeginMessage("/b/IPS.MARGIN_HORZONTAL.Value") << 0
        << osc::EndMessage
        << osc::BeginMessage("/b/IPS.MARGIN_VERTICAL.Value") << 0
        << osc::EndMessage;*/
    
    transmitSocket.Send( p.Data(), p.Size() );

    // Start listening for osc messages
    ExamplePacketListener listener;
    UdpListeningReceiveSocket s(
            IpEndpointName( IpEndpointName::ANY_ADDRESS, RECEIVING_PORT ),
            &listener );

    std::cout << "press ctrl-c to end\n";

    s.RunUntilSigInt();

    return 0;
}