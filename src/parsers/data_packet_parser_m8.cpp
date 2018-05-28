/**************************************************************** 
 **                                                            **
 **  Copyright(C) 2016 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <quanergy/parsers/data_packet_parser_m8.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/io/pcd_io.h>
#include <unordered_map>
#include <string>
#include <cstring>
#include <cmath>

#include <quanergy/osc/OscOutboundPacketStream.h>
#include <quanergy/osc/OscTypes.h>
#include <quanergy/ip/UdpSocket.h>
#include <quanergy/ip/IpEndpointName.h>
#include <quanergy/osc/OscReceivedElements.h>
#include <quanergy/osc/OscPacketListener.h>

#define ADDRESS "127.0.0.1"
#define RECEIVING_PORT 7000
#define SENDING_PORT 8000
#define OUTPUT_BUFFER_SIZE 1024

// class IpsPacketListener : public osc::OscPacketListener {
// protected:

//     virtual void ProcessMessage( const osc::ReceivedMessage& m, 
//         const IpEndpointName& remoteEndpoint )
//     {
//         (void) remoteEndpoint; // suppress unused parameter warning

//         try{
//             // example of parsing single messages. osc::OsckPacketListener
//             // handles the bundle traversal.
            
//             if( std::strcmp( m.AddressPattern(), "/test1" ) == 0 ){
//                 // example #1 -- argument stream interface
//                 osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
//                 bool a1;
//                 osc::int32 a2;
//                 float a3;
//                 const char *a4;
//                 args >> a1 >> a2 >> a3 >> a4 >> osc::EndMessage;
                
//                 std::cout << "received '/test1' message with arguments: "
//                     << a1 << " " << a2 << " " << a3 << " " << a4 << "\n";
                
//             }else if( std::strcmp( m.AddressPattern(), "/test2" ) == 0 ){
//                 // example #2 -- argument iterator interface, supports
//                 // reflection for overloaded messages (eg you can call 
//                 // (*arg)->IsBool() to check if a bool was passed etc).
//                 // osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
//                 // bool a1 = (arg++)->AsBool();
//                 // int a2 = (arg++)->AsInt32();
//                 // float a3 = (arg++)->AsFloat();
//                 // const char *a4 = (arg++)->AsString();
//                 // if( arg != m.ArgumentsEnd() )
//                 //     throw osc::ExcessArgumentException();
                
//                 // std::cout << "received '/test2' message with arguments: "
//                 //     << a1 << " " << a2 << " " << a3 << " " << a4 << "\n";
//             }
//         }catch( osc::Exception& e ){
//             // any parsing errors such as unexpected argument types, or 
//             // missing arguments get thrown as exceptions.
//             std::cout << "error while parsing message: "
//                 << m.AddressPattern() << ": " << e.what() << "\n";
//         }
//     }
// };

namespace quanergy
{
  namespace client
  {

    DataPacketParserM8::DataPacketParserM8()
      : DataPacketParser()
      , packet_counter_(0)
      , cloud_counter_(0)
      , last_azimuth_(65000)
      , current_cloud_(new PointCloudHVDIR())
      , worker_cloud_(new PointCloudHVDIR())
      , horizontal_angle_lookup_table_(M8_NUM_ROT_ANGLES+1)
      , start_azimuth_(0)
      , degrees_per_cloud_(361.0) // if this is greater than 360 we are getting a full scan
    {
      // Reserve space ahead of time for incoming data
      current_cloud_->reserve(maximum_cloud_size_);
      worker_cloud_->reserve(maximum_cloud_size_);

      for (std::uint32_t i = 0; i <= M8_NUM_ROT_ANGLES; i++)
      {
        // Shift by half the rot angles to keep the number positive when wrapping.
        std::uint32_t j = (i + M8_NUM_ROT_ANGLES/2) % M8_NUM_ROT_ANGLES;

        // normalized
        double n = static_cast<double>(j) / static_cast<double>(M8_NUM_ROT_ANGLES);

        double rad = n * M_PI * 2.0 - M_PI;

        horizontal_angle_lookup_table_[i] = rad;
      }

      const double* angle_in_radians = M8_VERTICAL_ANGLES;
      for (std::uint32_t i = 0; i < M8_NUM_LASERS; ++i, ++angle_in_radians)
      {
        vertical_angle_lookup_table_[i] = *angle_in_radians;
      }
    }

    void DataPacketParserM8::setReturnSelection(int return_selection)
    {
      if ((return_selection != quanergy::client::ALL_RETURNS) &&
          (return_selection < 0 || return_selection >= M8_NUM_RETURNS))
      {
        throw InvalidReturnSelection();
      }

      return_selection_ = return_selection;
    }

    void DataPacketParserM8::setCloudSizeLimits(std::int32_t szmin, std::int32_t szmax)
    {
      if(szmin > 0)
        minimum_cloud_size_ = std::max(1,szmin);
      if(szmax > 0)
        maximum_cloud_size_ = std::max(minimum_cloud_size_,szmax);
    }
    
    void DataPacketParserM8::setDegreesOfSweepPerCloud(double degrees_per_cloud)
    {
      if ( degrees_per_cloud < 0 || degrees_per_cloud > 360.0 ) 
      {
        throw InvalidDegreesPerCloud();
      }
      degrees_per_cloud_ = degrees_per_cloud;
    }

    bool DataPacketParserM8::parse(const M8DataPacket& data_packet, PointCloudHVDIRPtr& result)
    {
      bool ret = false;

      if (static_cast<StatusType>(data_packet.status) != StatusType::GOOD)
      {
        std::cerr << "Sensor status nonzero: " << data_packet.status;
        if (static_cast<std::uint16_t>(data_packet.status) & static_cast<std::uint16_t>(StatusType::SENSOR_SW_FW_MISMATCH))
        {
          throw FirmwareVersionMismatchError();
        }
        else if (static_cast<std::uint16_t>(data_packet.status) & static_cast<std::uint16_t>(StatusType::WATCHDOG_VIOLATION))
        {
          throw FirmwareWatchdogViolationError();
        }

        // Status flag is set, but is not currently known in this version of the software
        throw FirmwareUnknownError();
      }

      // get the timestamp of the last point in the packet as 64 bit integer in units of microseconds
      std::uint64_t current_packet_stamp ;
      if (data_packet.version <= 3 && data_packet.version != 0)
      {
        // some versions of API put 10 ns increments in this field
        current_packet_stamp = data_packet.seconds * 1E6 + data_packet.nanoseconds * 1E-2;
      }
      else
        current_packet_stamp = data_packet.seconds * 1E6 + data_packet.nanoseconds * 1E-3;

      if (previous_packet_stamp_ == 0)
      {
        previous_packet_stamp_ = current_packet_stamp;
      }

      ++packet_counter_;

      // get spin direction
      // check 3 points in the packet to figure out which way it is spinning
      // if the measurements disagree, it could be wrap so we'll ignore that
      if (data_packet.data[0].position - data_packet.data[M8_FIRING_PER_PKT/2].position < 0
          && data_packet.data[M8_FIRING_PER_PKT/2].position - data_packet.data[M8_FIRING_PER_PKT-1].position < 0)
      {
        direction_ = 1;
      }
      else if (data_packet.data[0].position - data_packet.data[M8_FIRING_PER_PKT/2].position > 0
          && data_packet.data[M8_FIRING_PER_PKT/2].position - data_packet.data[M8_FIRING_PER_PKT-1].position > 0)
      {
        direction_ = -1;
      }

      double distance_scaling = 0.01f;
      if (data_packet.version >= 5)
      {
        distance_scaling = 0.00001f;
      }

      bool cloudfull = (current_cloud_->size() >= maximum_cloud_size_);

      // for each firing
      for (int i = 0; i < M8_FIRING_PER_PKT; ++i)
      {
        const M8FiringData &data = data_packet.data[i];

        // calculate the angle in degrees
        double azimuth_angle = (static_cast<double> ((data.position+(M8_NUM_ROT_ANGLES/2))%M8_NUM_ROT_ANGLES) / (M8_NUM_ROT_ANGLES) * 360.0) - 180.;
        double delta_angle = 0;
        if ( cloud_counter_ == 0 && start_azimuth_ == 0 )
        {
          start_azimuth_ = azimuth_angle;
        } 
        else 
        {
          // calculate delta
          delta_angle = direction_*(azimuth_angle - start_azimuth_);
          while ( delta_angle < 0.0 )
          {
            delta_angle += 360.0;
          }
        }

        //std::cout << "delta_angle: " << delta_angle << std::endl;
        
        if ( delta_angle >= degrees_per_cloud_ || (degrees_per_cloud_==360.0 && (direction_*azimuth_angle < direction_*last_azimuth_) ))
        {
          start_azimuth_ = azimuth_angle;
          if (current_cloud_->size () > minimum_cloud_size_)
          {
            if(cloudfull)
            {
              std::cout << "Warning: Maximum cloud size limit of ("
                  << maximum_cloud_size_ << ") exceeded" << std::endl;
            }

						// interpolate the timestamp from the previous packet timestamp to the timestamp of this firing
            const double time_since_previous_packet =
                (current_packet_stamp - previous_packet_stamp_) * i / static_cast<double>(M8_FIRING_PER_PKT);
            const auto current_firing_stamp = static_cast<uint64_t>(std::round(
                previous_packet_stamp_ + time_since_previous_packet));

            current_cloud_->header.stamp = current_firing_stamp;
            current_cloud_->header.seq = cloud_counter_;
            current_cloud_->header.frame_id = frame_id_;

            // can't organize if we kept all points
            if (return_selection_ != quanergy::client::ALL_RETURNS)
            {
              organizeCloud(current_cloud_, worker_cloud_);
            }

            ++cloud_counter_;

            // fire the signal that we have a new cloud
            result = current_cloud_;
            ret = true;
          }
          else if(current_cloud_->size() > 0)
          {
            std::cout << "Warning: Minimum cloud size limit of (" << minimum_cloud_size_
                << ") not reached (" << current_cloud_->size() << ")" << std::endl;
          }

          // BEGINNING OF OUR CODE

          static std::unordered_map<std::string, std::vector<double>> base_scan = {};
          static std::unordered_map<std::string, std::vector<double>> comp_scan = {};
          static int visits_here = 0;
          static float origin_baseline_distance = 0;
          static float origin_baseline_intensity = 0;
          static float origin_distance = 0;
          static float origin_intensity = 0;


          if (visits_here == 5) {
            for (PointCloudHVDIR::const_iterator i = current_cloud_->begin(); i != current_cloud_->end(); ++i) {
              std::vector<double> v;
              v.push_back(i->h);
              v.push_back(i->v);
              v.push_back(i->d);
              v.push_back(i->intensity);
              base_scan[std::to_string(floor(i->h * 500 + 0.005) / 500) + std::to_string(floor(i->v * 500 + 0.005) / 500)] = v;

              // Get intensity of origin
              if (i->h > -0.0015 && i->h < 0.0015 && i->v > -0.0015 && i->v < 0.0015) {
                std::cout << "setting baseline origin intensity" << std::endl;
                origin_baseline_intensity = i->intensity;
                origin_baseline_distance = i->d;

                UdpTransmitSocket transmitSocket( IpEndpointName( ADDRESS, SENDING_PORT ) );
                  char buffer[OUTPUT_BUFFER_SIZE];
                osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
                p << osc::BeginBundleImmediate
                    //<< osc::BeginMessage("/b/Beam.0.Power") << (int)(40)
                    //<< osc::EndMessage
                    //<< osc::BeginMessage("/b/VARIABLES.origin_baseline_distance") << (origin_baseline_distance)
                    //<< osc::EndMessage
                    //<< osc::BeginMessage("/b/VARIABLES.origin_baseline_intensity") << (origin_baseline_intensity)
                    //<< osc::EndMessage
                  << osc::EndBundle;
            
                  transmitSocket.Send( p.Data(), p.Size() );
              }
            }
          }

          if (visits_here > 5) {
            for (PointCloudHVDIR::const_iterator i = current_cloud_->begin(); i != current_cloud_->end(); ++i) {
              std::vector<double> v;
              v.push_back(i->h);
              v.push_back(i->v);
              v.push_back(i->d);
              v.push_back(i->intensity);
              comp_scan[std::to_string(floor(i->h * 500 + 0.005) / 500) + std::to_string(floor(i->v * 500 + 0.005) / 500)] = v;
              
              //std::cout << "baseline distance, "<< origin_baseline_distance << ", baseline intensity, " << origin_baseline_intensity 
              //<<  ", distance, " << origin_distance<< ", intensity, " << origin_intensity << std::endl;

               //Check intensity of origin
               //If intensity has changed more than 10% - do something
              if (i->h > -0.0015 && i->h < 0.0015 && i->v > -0.0015 && i->v < 0.0015) {
                origin_distance = i->d;
                origin_intensity = i->intensity;
                if (std::abs(i->intensity - origin_baseline_intensity) > (origin_baseline_intensity*.50)) {
                  
                if (std::abs(origin_intensity - origin_baseline_intensity) > (origin_baseline_intensity*.50)) {
                  UdpTransmitSocket transmitSocket( IpEndpointName( ADDRESS, SENDING_PORT ) );
                  char buffer[OUTPUT_BUFFER_SIZE];
                  osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
            
                  p << osc::BeginBundleImmediate
                    //<< osc::BeginMessage("/b/Beam.0.Power") << (int)(40)
                    //<< osc::EndMessage
                    //<< osc::BeginMessage("/b/CALIBRATION.DETECTED.Value") << (int)(1)
                    //<< osc::EndMessage
                  << osc::EndBundle;
            
                  transmitSocket.Send( p.Data(), p.Size() );
                }
                else {
                  UdpTransmitSocket transmitSocket( IpEndpointName( ADDRESS, SENDING_PORT ) );
                  char buffer[OUTPUT_BUFFER_SIZE];
                  osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
            
                  p << osc::BeginBundleImmediate
                    //<< osc::BeginMessage("/b/Beam.0.Power") << (int)(10)
                    //<< osc::EndMessage
                    //<< osc::BeginMessage("/b/VARIABLES.origin_baseline_distance") << (origin_baseline_distance)
                    //<< osc::EndMessage
                    //<< osc::BeginMessage("/b/VARIABLES.origin_baseline_intensity") << (origin_baseline_intensity)
                    //<< osc::EndMessage
                    //<< osc::BeginMessage("/b/VARIABLES.origin_distance") << (origin_distance)
                    //<< osc::EndMessage
                    //<< osc::BeginMessage("/b/VARIABLES.origin_intensity") << (origin_intensity)
                    //<< osc::EndMessage
                    //<< osc::BeginMessage("/b/CALIBRATION.DETECTED.Value") << (int)(0)
                    //<< osc::EndMessage
                  << osc::EndBundle;
            
                  transmitSocket.Send( p.Data(), p.Size() );} 
                }
              }
            }
              
            // Defining Exlusion Zone variables
            int count = 0;
            double max_h = -100;
            double min_h = 100;
            double max_v = -100;
            double min_v = 100;
            double max_d = 1;
            double min_d = 200;
            double max_h_distance = 1;
            double min_h_distance = 1;
            double max_h_intensity;
            double min_h_intensity;
            double max_v_distance = 1;
            double min_v_distance = 1;
            double max_v_intensity;
            double min_v_intensity;

            //Define feild of view variables
            double fov_min_h;
            double fov_min_h_d=1;
            double fov_min_h_i;
            double fov_max_h;
            double fov_max_h_d=1;
            double fov_max_h_i;
            double fov_min_v; //= -0.3185;
            double fov_min_v_d=1;
            double fov_min_v_i;
            double fov_max_v; //= 0.0558;
            double fov_max_v_d=1;
            double fov_max_v_i;
            double fov_center_d=2;
            double fov_center_i;

            // Recording exclusion zone and field of view
            for (auto i = comp_scan.begin(); i != comp_scan.end(); ++i) {
              
              // Get fov min_h data
              if (i == comp_scan.begin()) {
                fov_min_h = i->second[0];
                fov_min_h_d = i->second[2];
                fov_min_h_i = i->second[3];
              }

              // Get fov max_h data
              if (i == comp_scan.end()) {
                fov_max_h = i->second[0];
                fov_max_h_d = i->second[2];
                fov_max_h_i = i->second[3];
              }
              
              // Get fov min_v data
              if (i == comp_scan.begin()) {
                fov_min_v = i->second[1];
                fov_min_v_d = i->second[2];
                fov_max_v_i = i->second[3];
              }
              // Get fov max_v data
              if (i == comp_scan.end()) {
                fov_max_v = i->second[1];
                fov_max_v_d = i->second[2];
                fov_max_v_i = i->second[3];
              }

              // Get center data
              //if (i->h > -0.0015 && i->h < 0.0015 && i->v == -0.165195){
              if (i->second[0]>-0.005 && i->second[0] <0.005 && i->second[1] == -0.165195) {
                fov_center_d = i->second[2];
                fov_center_i = i->second[3];
              }

              // Get exclusion zone data
              if (base_scan.find(i->first) != base_scan.end()) {
                double distance = base_scan.find(i->first)->second[2];                  
                if (std::abs(i->second[2] - distance) > distance *0.04) {
                  count++;
                  if (i->second[0] > max_h) {
                    max_h = i->second[0];
                    max_h_distance = i->second[2];
                    max_h_intensity = i->second[3];
                  } 
                  if (i->second[0] < min_h) {
                    min_h = i->second[0];
                    min_h_distance = i->second[2];
                    min_h_intensity = i->second[3];
                  }
                  if (i->second[1] > max_v) {
                    max_v = i->second[1];
                    max_v_distance = i->second[2];
                    max_v_intensity = i->second[3];
                  } 
                  if (i->second[1] < min_v) {
                    min_v = i->second[1];
                    min_v_distance = i->second[2];
                    min_v_intensity = i->second[3];
                  }
                  if (i->second[2] > max_d) {
                    max_d = i->second[2];
                  } 
                  if (i->second[2] < min_d) {
                    min_d = i->second[2];
                  }
                }
              }
            }

            // Switching max and min
            // Right of center is negative, left of center is positive
            std::cout << "count: " << count << ", min_h " << min_h << ", max_h " << max_h << ", fov_min_h " <<fov_min_h<< ", fov_max_h " <<fov_max_h<< std::endl;
            auto temp = min_h;
            min_h = max_h*-1;
            max_h = temp*-1;
            temp = fov_min_h;
            fov_min_h = fov_max_h*-1;
            fov_max_h = temp*-1;
            std::cout << "count: " << count << ", min_h " << min_h << ", max_h " << max_h << ", fov_min_h " <<fov_min_h<< ", fov_max_h " <<fov_max_h<< std::endl;

            // Margin setup
            double margine_x = 0; // Margine width in meters
            double margine_y = 0;
            double max_vm = max_v+0.0567+atan(margine_y/max_v_distance); // add angle (0.0567 radians, 3.25 degrees) to protect the space before next empty ring and add margin angle in radians
            double min_vm = min_v-0.0567-atan(margine_y/min_v_distance); //-0.0567


            // Sensor offset compensation (vertical offset only)
            double offset_y = 0.195; //height of sensor origin above scanner origin in meters (measured 0.195 +/-0.002)
            double max_vt = atan((offset_y+max_v_distance*sin(max_vm))/(max_v_distance*cos(max_vm))); //v transformed to projector origin
            double min_vt = atan((offset_y+min_v_distance*sin(min_vm))/(min_v_distance*cos(min_vm)));
            double max_yc = 191*(atan((offset_y+max_v_distance*sin(max_v))/(max_v_distance*cos(max_v)))); 
            double min_yc = 191*(atan((offset_y+min_v_distance*sin(min_v))/(min_v_distance*cos(min_v))));
            double max_xc = max_h*191;
            double min_xc = min_h*191;
            double avg_xc = min_xc+((max_xc-min_xc)/2);
            double avg_yc = min_yc+((max_yc-min_yc)/2);
            
            // Scale for beyond software
            // Projector field of view is roughly -30 t0 +30 degrees
            // Beyond software window clipout effect value range is -100 to 100
            // x = (h+margine)*(180\PI)*(100\30)
            double max_x = (max_h+margine_x)*191;
            double min_x = (min_h-margine_x)*191;
            double min_y;
            if (min_v < -0.3) {min_y = -110;}
            if (min_v > -0.3) {min_y = min_vt*191;} 
            double max_y;
            if (max_v > 0.04) {max_y = 110;}
            if (max_v < 0.04) {max_y = max_vt*191;} 

            //std::cout << "count: " << count << ", fov_max_v: " << fov_max_v << ", fov_min_v: " << fov_min_v<< ", fov_max_v_distance: " << fov_max_v_distance << ", fov_min_v_distance: " << fov_min_v_distance << std::endl;
            //std::cout << "count: " << count << ", fov_centter_d " << fov_center_d << ", max_v " << max_v << std::endl;
            
            UdpTransmitSocket transmitSocket( IpEndpointName( ADDRESS, SENDING_PORT ) );
            char buffer[OUTPUT_BUFFER_SIZE];
            osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
            
            p << osc::BeginBundleImmediate
                //PROTECT 1
                // << osc::BeginMessage("/b/IPS/PROTECT_1.Effect.0/Keys.0/Value1") << (float)(min_x)
                // << osc::EndMessage
                // << osc::BeginMessage("/b/IPS/PROTECT_1.Effect.0/Keys.0/Value2") << (float)(max_x)
                // << osc::EndMessage
                // << osc::BeginMessage("/b/IPS/PROTECT_1.Effect.0/Keys.0/Value3") << (float)(min_y)
                // << osc::EndMessage
                // << osc::BeginMessage("/b/IPS/PROTECT_1.Effect.0/Keys.0/Value4") << (float)(max_y)
                // << osc::EndMessage
                
                // Field of view data
                << osc::BeginMessage("/b/VARIABLES.fov_min_h") << (float)(fov_min_h)
                << osc::EndMessage
                //<< osc::BeginMessage("/b/VARIABLES.fov_min_h_d") << (float)(fov_min_h_d)
                // << osc::EndMessage
                // << osc::BeginMessage("/b/VARIABLES.fov_min_h_i") << (float)(fov_min_h_i)
                // << osc::EndMessage
                << osc::BeginMessage("/b/VARIABLES.fov_max_h") << (float)(fov_max_h)
                << osc::EndMessage
                //<< osc::BeginMessage("/b/VARIABLES.fov_max_h_d") << (float)(fov_max_h_d)
                // << osc::EndMessage
                // << osc::BeginMessage("/b/VARIABLES.fov_max_h_i") << (float)(fov_max_h_i)
                // << osc::EndMessage
                //<< osc::BeginMessage("/b/VARIABLES.fov_min_v") << (float)(fov_min_v)
                // << osc::EndMessage
                //<< osc::BeginMessage("/b/VARIABLES.fov_max_v") << (float)(fov_max_v)
                // << osc::EndMessage
                << osc::BeginMessage("/b/SCANFIELD.fov_center_d") << (float)(fov_center_d)
                << osc::EndMessage
                // << osc::BeginMessage("/b/VARIABLES.fov_center_i") << (float)(fov_center_i)
                // << osc::EndMessage

                // Exclusion zone data
                << osc::BeginMessage("/b/CALIBRATIONDYNAMIC.PositionX") << (int)(max_xc)
                << osc::EndMessage
                << osc::BeginMessage("/b/CALIBRATIONDYNAMIC.PositionY") << (int)(max_yc)
                << osc::EndMessage
                << osc::BeginMessage("/b/VARIABLES.max_v") << (float)(max_v)
                << osc::EndMessage
                << osc::BeginMessage("/b/VARIABLES.min_v") << (float)(min_v)
                << osc::EndMessage
                << osc::BeginMessage("/b/VARIABLES.min_h") << (float)(min_h)
                << osc::EndMessage
                << osc::BeginMessage("/b/VARIABLES.max_h") << (float)(max_h)
                << osc::EndMessage
                << osc::BeginMessage("/b/VARIABLES.max_v_d") << (float)(max_v_distance)
                << osc::EndMessage
                << osc::BeginMessage("/b/VARIABLES.min_v_d") << (float)(min_v_distance)
                << osc::EndMessage
                << osc::BeginMessage("/b/VARIABLES.min_h_d") << (float)(min_h_distance)
                << osc::EndMessage
                << osc::BeginMessage("/b/VARIABLES.max_h_d") << (float)(max_h_distance)
                << osc::EndMessage
                << osc::BeginMessage("/b/VARIABLES.origin_baseline_distance") << (origin_baseline_distance)
                << osc::EndMessage
                << osc::BeginMessage("/b/VARIABLES.origin_baseline_intensity") << (origin_baseline_intensity)
                << osc::EndMessage
                << osc::BeginMessage("/b/VARIABLES.origin_distance") << (origin_distance)
                << osc::EndMessage
                << osc::BeginMessage("/b/VARIABLES.origin_intensity") << (origin_intensity)
                << osc::EndMessage
                << osc::EndBundle;
            
            transmitSocket.Send( p.Data(), p.Size() );

            // IpsPacketListener listener;
            // UdpListeningReceiveSocket s(
            //   IpEndpointName( IpEndpointName::ANY_ADDRESS, RECEIVING_PORT ),
            //   &listener
            // );

              /*
              std::cout << "comparison original -------------------------------------";
              for (PointCloudHVDIR::const_iterator i = comparison_scan.begin(); i != comparison_scan.end(); ++i) {
                  if (i->ring == 0) {
                      std::cout << i->h << std::endl;
                  }
              }*/

              //const auto base_scan_B = base_scan;

              //boost::shared_ptr<pcl::search::Search<quanergy::PointHVDIR>> x = boost::shared_ptr<pcl::search::Search<quanergy::PointHVDIR>>();
              //pcl::PointCloud<quanergy::PointHVDIR> y = pcl::PointCloud<quanergy::PointHVDIR>();

              //pcl::getPointCloudDifference(base_scan, comparison_scan, 0.001, x, y);

            // Round angles in both scans to nearest thousandth radian
            //std::cout << "base rounded -------------------------------------";


            /*
            std::cout << "comparison rounded -------------------------------------";
            for (PointCloudHVDIR::const_iterator i = comparison_scan.begin(); i != comparison_scan.end(); ++i) {
                if (i->ring == 0) {
                    std::cout << (floor(i->h * 500 + 0.005)) / 500 << std::endl;
                }
            }*/

              // for (PointCloudHVDIR::const_iterator i = base_scan.begin(); i != base_scan.end(); ++i) {
              //     if (i->ring == 0) {
              //         std::cout << i->h << std::endl;
              //     }
              // }
              // for (PointCloudHVDIR::const_iterator i = comparison_scan.begin(); i != comparison_scan.end(); ++i) {
              //     if (i->ring == 0) {
              //         std::cout << i->h << std::endl;
              //     }
              // }
          }
          visits_here++;

          // Get difference in distances between base and comparison scan
          //auto comparison_scan = PointCloudHVDIR(); 
          //for (PointCloudHVDIR::const_iterator i = base_scan.begin(); i != base_scan.end(); ++i) {
          //    i->h = Math.round(i->h);
          //}

          // start a new cloud
          current_cloud_.reset(new PointCloudHVDIR());
          // at first we assume it is dense
          current_cloud_->is_dense = true;
          current_cloud_->reserve(maximum_cloud_size_);
          cloudfull = false;
        }

        if(cloudfull)
          continue;

        double const horizontal_angle = horizontal_angle_lookup_table_[data.position];

        for (int j = 0; j < M8_NUM_LASERS; j++)
        {
          // output point
          PointCloudHVDIR::PointType hvdir;

          double const vertical_angle = vertical_angle_lookup_table_[j];

          hvdir.h = horizontal_angle;
          hvdir.v = vertical_angle;
          hvdir.ring = j;

          if (return_selection_ == quanergy::client::ALL_RETURNS)
          {
            // for the all case, we won't keep NaN points and we'll compare
            // distances to illiminate duplicates
            // index 0 (max return) could equal index 1 (first) and/or index 2 (last)
            hvdir.intensity = data.returns_intensities[0][j];
            std::uint32_t d = data.returns_distances[0][j];
            if (d != 0)
            {
              hvdir.d = static_cast<float>(d) * distance_scaling; // convert range to meters
              // add the point to the current scan
              current_cloud_->push_back(hvdir);
            }

            if (data.returns_distances[1][j] != 0 && data.returns_distances[1][j] != d)
            {
              hvdir.intensity = data.returns_intensities[0][j];
              hvdir.d = static_cast<float>(data.returns_distances[1][j]) * distance_scaling; // convert range to meters
              // add the point to the current scan
              current_cloud_->push_back(hvdir);
            }

            if (data.returns_distances[2][j] != 0 && data.returns_distances[2][j] != d)
            {
              hvdir.intensity = data.returns_intensities[0][j];
              hvdir.d = static_cast<float>(data.returns_distances[2][j]) * distance_scaling; // convert range to meters
              // add the point to the current scan
              current_cloud_->push_back(hvdir);
            }
          }
          else
          {
            for (int i = 0; i < quanergy::client::M8_NUM_RETURNS; ++i)
            {
              if (return_selection_ == i)
              {
                hvdir.intensity = data.returns_intensities[i][j];

                if (data.returns_distances[i][j] == 0)
                {
                  hvdir.d = std::numeric_limits<float>::quiet_NaN();
                  // if the range is NaN, the cloud is not dense
                  current_cloud_->is_dense = false;
                }
                else
                {
                  hvdir.d = static_cast<float>(data.returns_distances[i][j]) * distance_scaling; // convert range to meters
                }

                // add the point to the current scan
                current_cloud_->push_back(hvdir);
              }
            }
          }
        }

        last_azimuth_ = azimuth_angle;
      }

      previous_packet_stamp_ = current_packet_stamp;

      return ret;
    }

    void DataPacketParserM8::organizeCloud(PointCloudHVDIRPtr & current_pc,
                                           PointCloudHVDIRPtr & temp_pc)
    {
      // transpose the cloud
      temp_pc->clear();

      temp_pc->header.stamp = current_pc->header.stamp;
      temp_pc->header.seq = current_pc->header.seq;
      temp_pc->header.frame_id = current_pc->header.frame_id;

      // reserve space
      temp_pc->reserve(current_pc->size());

      unsigned int temp_index;
      unsigned int width = current_pc->size () / M8_NUM_LASERS; // CONSTANT FOR NUM BEAMS

      // iterate through each ring from top down
      for (int i = M8_NUM_LASERS - 1; i >= 0; --i)
      {
        // iterate through width in collect order
        for (unsigned int j = 0; j < width; ++j)
        {
          // original data is in collect order and laser order
          temp_index = j * M8_NUM_LASERS + i;

          temp_pc->push_back(current_pc->points[temp_index]);
        }
      }

      current_pc.swap(temp_pc);

      current_pc->height = M8_NUM_LASERS;
      current_pc->width  = width;
    }

  } // namespace client

} // namespace quanergy
