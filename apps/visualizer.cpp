/****************************************************************
 **                                                            **
 **  Copyright(C) 2016 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

// visualization module
#include "visualizer_module.h"

// console parser
#include <pcl/console/parse.h>

#include <quanergy/client/sensor_client.h>

// parsers for the data packets we want to support
#include <quanergy/parsers/data_packet_parser_00.h>
#include <quanergy/parsers/data_packet_parser_01.h>
#include <quanergy/parsers/data_packet_parser_04.h>
#include <quanergy/parsers/variadic_packet_parser.h>

// module to apply encoder correction
#include <quanergy/modules/encoder_angle_calibration.h>

// conversion module from polar to Cartesian
#include <quanergy/modules/polar_to_cart_converter.h>

#include <quanergy/osc/OscTypes.h>
#include <quanergy/ip/UdpSocket.h>
#include <quanergy/ip/IpEndpointName.h>
#include <quanergy/osc/OscReceivedElements.h>
#include <quanergy/osc/OscPacketListener.h>

#define ADDRESS "127.0.0.1"
#define RECEIVING_PORT 7000
#define SENDING_PORT 8000
#define OUTPUT_BUFFER_SIZE 1024

namespace
{
  static const std::string MANUAL_CORRECT{"--manual-correct"};
  static const std::string CALIBRATE_STR{"--calibrate"};
  static const std::string AMPLITUDE_STR{"--amplitude"};
  static const std::string PHASE_STR{"--phase"};
}

void usage(char** argv)
{
  std::cout << "usage: " << argv[0]
      << " --host <host> [-h | --help] [" << CALIBRATE_STR << "][" << MANUAL_CORRECT << " " << AMPLITUDE_STR << " <value> " << PHASE_STR << " <value>]" << std::endl << std::endl

      << "    --host        hostname or IP address of the sensor" << std::endl
      << "    " << CALIBRATE_STR << "   calibrate the host sensor and apply calibration to outgoing points" << std::endl
      << "    " << MANUAL_CORRECT << " --amplitude <amplitude> --phase <phase>    Manually correct encoder error specifying amplitude and phase correction, in radians" << std::endl
      << "-h, --help        show this help and exit" << std::endl;
  return;
}

typedef quanergy::client::SensorClient ClientType;
typedef quanergy::client::VariadicPacketParser<quanergy::PointCloudHVDIRPtr,                      // return type
                                               quanergy::client::DataPacketParser00,              // PARSER_00_INDEX
                                               quanergy::client::DataPacketParser01,              // PARSER_00_INDEX
                                               quanergy::client::DataPacketParser04> ParserType;  // PARSER_04_INDEX
enum
{
  PARSER_00_INDEX = 0,
  PARSER_01_INDEX = 1,
  PARSER_04_INDEX = 2
};

typedef quanergy::client::PacketParserModule<ParserType> ParserModuleType;
typedef quanergy::calibration::EncoderAngleCalibration CalibrationType;
typedef quanergy::client::PolarToCartConverter ConverterType;

class ExamplePacketListener : public osc::OscPacketListener {
protected:

    virtual void ProcessMessage( const osc::ReceivedMessage& m, 
        const IpEndpointName& remoteEndpoint )
    {
        (void) remoteEndpoint; // suppress unused parameter warning

        try {
            // example of parsing single messages. osc::OsckPacketListener
            // handles the bundle traversal.
            
            if ( std::strcmp( m.AddressPattern(), "/b/IPS.LIDAR_START.Value" ) == 0 ) {
                osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
                float a3;
                args >> a3 >> osc::EndMessage;
                
                if (a3 == 1) {
                    std::system("~/quanergy_client/build/visualizer --host 192.168.88.12 &");
                } else if (a3 == 0) {
                    std::system("killall visualizer");
                } else {

                }
            }
        } catch( osc::Exception& e ) {
            // any parsing errors such as unexpected argument types, or 
            // missing arguments get thrown as exceptions.
            std::cout << "error while parsing message: "
                << m.AddressPattern() << ": " << e.what() << "\n";
        }
    }
};

int main(int argc, char** argv)
{
  int max_num_args = 8;
  // get host
  if (argc < 2 || argc > max_num_args || pcl::console::find_switch(argc, argv, "-h") ||
      pcl::console::find_switch(argc, argv, "--help") || !pcl::console::find_switch(argc, argv, "--host"))
  {
    usage (argv);
    return (0);
  }

  std::string host;
  std::string port = "4141";

  pcl::console::parse_argument(argc, argv, "--host", host);

  // create modules
  ClientType client(host, port, 100);
  ParserModuleType parser;
  ConverterType converter;
  //VisualizerModule visualizer;

  CalibrationType calibrator;

  // setup modules
  parser.get<PARSER_00_INDEX>().setFrameId("quanergy");
  parser.get<PARSER_00_INDEX>().setReturnSelection(0);
  parser.get<PARSER_00_INDEX>().setDegreesOfSweepPerCloud(360.0);
  parser.get<PARSER_01_INDEX>().setFrameId("quanergy");
  parser.get<PARSER_04_INDEX>().setFrameId("quanergy");

  // connect modules
  std::vector<boost::signals2::connection> connections;

  connections.push_back(client.connect([&parser](const ClientType::ResultType& pc){ parser.slot(pc); }));
  
  // if we're doing automatic calibration or if we're setting the calibration
  // manually, include the calibrator in the chain
  if (pcl::console::find_switch(argc, argv, CALIBRATE_STR.c_str()) ||
      pcl::console::find_switch(argc, argv, MANUAL_CORRECT.c_str()))
  {
    connections.push_back(parser.connect([&calibrator](const ParserModuleType::ResultType& pc){ calibrator.slot(pc); }));
    connections.push_back(calibrator.connect([&converter](const CalibrationType::ResultType& pc){ converter.slot(pc); }));

    if (pcl::console::find_switch(argc, argv, MANUAL_CORRECT.c_str()))
    {
      if (!pcl::console::find_switch(argc, argv, AMPLITUDE_STR.c_str()) ||
          !pcl::console::find_switch(argc, argv, PHASE_STR.c_str()))
      {
        usage(argv);
        return(0);
      }
      double amplitude = 0.;
      double phase = 0.;
      pcl::console::parse_argument(argc, argv, AMPLITUDE_STR.c_str(), amplitude);
      pcl::console::parse_argument(argc, argv, PHASE_STR.c_str(), phase);

      calibrator.setParams(amplitude, phase);
    }
  }
  else
  {
    connections.push_back(parser.connect([&converter](const ParserModuleType::ResultType& pc){ converter.slot(pc); }));
  }

  //connections.push_back(converter.connect([&visualizer](const ConverterType::ResultType& pc){ visualizer.slot(pc); }));

  // run the client with the calibrator and wait for a signal from the
  // calibrator that a successful calibration has been performed
  std::thread client_thread([&client/*, &visualizer*/]
  {
    try
    {
      client.run();
      
    }
    catch (std::exception& e)
    {
      std::cerr << "Terminating after catching exception: " << e.what() << std::endl;
      //visualizer.stop();
    }
  });
    
  // start visualizer (blocks until stopped)
  //visualizer.run();

  // clean up
  client.stop();
  connections.clear();
  client_thread.join();

  return (0);
}
