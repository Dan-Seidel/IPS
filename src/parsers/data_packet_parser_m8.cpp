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
#include <fstream>
#include <stdio.h>

#include <quanergy/osc/OscOutboundPacketStream.h>
#include <quanergy/osc/OscTypes.h>
#include <quanergy/ip/UdpSocket.h>
#include <quanergy/ip/IpEndpointName.h>

#define ADDRESS "127.0.0.1"
#define SENDING_PORT 8000
#define OUTPUT_BUFFER_SIZE 2048 //DEFAULT WAS 1024

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

          /////////////////////////// INTERACTIVE PROJECTION SYSTEM CODE/////////////////////////////////
          static std::unordered_map<std::string, std::vector<double>> base_scan = {};
          static std::unordered_map<std::string, std::vector<double>> comp_scan = {};
          static int visits_here = 0;
          static float origin_baseline_distance = 0;
          static float origin_baseline_intensity = 0;
          static float origin_distance = 0;
          static float origin_intensity = 0;
          
          static int base_scans_captured = 0;
          static int scan_status = 0;

          std::ifstream file("../../setBaseline.txt");

          if (file) {
            int total_points = 0;
            // Get baseline scan
            for (PointCloudHVDIR::const_iterator i = current_cloud_->begin(); i != current_cloud_->end(); ++i) {
              ++total_points;
              if (i->d > 0.1 && i->d < 500) {
                std::vector<double> v;
                v.push_back(i->h);
                v.push_back(i->v);
                v.push_back(i->d);
                v.push_back(i->intensity);
                v.push_back(i->ring);
                base_scan[std::to_string(floor(i->h * 10000) / 10000) + std::to_string(floor(i->v * 500 + 0.005) / 500)] = v;
                //std::cout << i->h << " " << floor(i->h * 10000) / 10000 << std::endl;
              }            

              // Get intensity of origin
              if (i->h > -0.0010 && i->h < 0.0010 && i->v == 0) {
                std::cout << "setting baseline origin intensity" << std::endl;
                origin_baseline_intensity = i->intensity;
                origin_baseline_distance = i->d;
              }
            }
            scan_status = 0;
            ++base_scans_captured;
            std::cout << base_scans_captured << " base scans captured" << std::endl;
            if (base_scans_captured > 100) {
              std::remove("../../setBaseline.txt");
              /*if (total_points != 0) {
                std::cout << base_scan.size() / (double)total_points * 100 << "%" << std::endl;
              }*/
              scan_status = 1;
              base_scans_captured = 0;
              std::cout << "done with base scan capturing" << std::endl;
              
            }
          }

          if (visits_here > 5) {
            int total_points = 0;
            int updated = 0, created = 0;
            // Get comparison scan
            for (PointCloudHVDIR::const_iterator i = current_cloud_->begin(); i != current_cloud_->end(); ++i) {
              ++total_points;
              // this conditional throws out nan points
              if (i->d > 0.1 && i->d < 500) {
                //std::vector<double> v = comp_scan[std::to_string(floor(i->h * 10000) / 10000) + std::to_string(floor(i->v * 500 + 0.005) / 500)];
                std::string key = std::to_string(floor(i->h * 10000) / 10000) + std::to_string(floor(i->v * 500 + 0.005) / 500);
                if (comp_scan.find(key) != comp_scan.end()) {
                  //std::cout << v[2] << " - " << i->d << std::endl;
                  comp_scan[key][2] = i->d;
                  comp_scan[key][3] = i->intensity;
                  //comp_scan->second[5] = visits_here;                  
                  //std::cout << "updating timestamp from " << v[5] << " to " << visits_here << std::endl;
                  /*updated++;
                  if (v[0] > -0.005 && v[0] < 0.005 && v[4] == 3) {
                    std::cout << "updating a point that should update fov_center3, from " << v[2] << " to " << i->d << std::endl;
                  }
                  if (v[0] == 0 && v[4] == 6) {
                    std::cout << "UPDATE point - h:" << v[0] << ", v:" << v[1] << ", d:" << v[2] << ", i:" << v[3] << ", r:" << v[4] << ", t:" << v[5] << ", i->d: " << i->d << std::endl;
                  }*/
                } else {
                  std::vector<double> v;
                  v.push_back(i->h);
                  v.push_back(i->v);
                  v.push_back(i->d);
                  v.push_back(i->intensity);
                  v.push_back(i->ring);
                  v.push_back(0);
                  //v.push_back(visits_here);
                  comp_scan[std::to_string(floor(i->h * 10000) / 10000) + std::to_string(floor(i->v * 500 + 0.005) / 500)] = v;
                  /*created++;
                  if (v[0] == 0 && v[4] == 6) {
                    std::cout << "CREATED point - h:" << v[0] << ", v:" << v[1] << ", d:" << v[2] << ", i:" << v[3] << ", r:" << v[4] << ", t:" << v[5] << std::endl;
                  }*/
                }
              }

              // If origin intensity has changed more than 10% - do something
              if (i->h > -0.0015 && i->h < 0.0015 && i->v > -0.0015 && i->v < 0.0015) {
                origin_distance = i->d;
                origin_intensity = i->intensity;
              }
            }
            //std::cout << "origin_distance:" << origin_distance << ", origin_intensity:" << origin_intensity << std::endl;

            /*for (auto i = comp_scan.begin(); i != comp_scan.end(); ++i) {
              if (i->second[0] == 0 && i->second[4] == 6) {
                std::cout << "PEEKING point - h:" << i->second[0] << ", v:" << i->second[1] << ", d:" << i->second[2] << ", i:" << i->second[3] << ", r:" << i->second[4] << ", t:" << i->second[5] << std::endl;
              }
            }*/

            //std::cout << "size: " << comp_scan.size() << ", updated: " << updated << ", created: " << created << std::endl;
        
            /*if (total_points != 0) {
              std::cout << comp_scan.size() << " " << (double)total_points << std::endl;
              std::cout << comp_scan.size() / (double)total_points * 100 << "%" << std::endl;
            }*/
              
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

            // Define field of view variables
            double fov_min_h=1;
            double fov_min_h_d=1;
            double fov_min_h_i;
            double fov_max_h=-1;
            double fov_max_h_d=1;
            double fov_max_h_i;
            double fov_min_v; //= -0.3185;
            double fov_min_v_d=1;
            double fov_min_v_i;
            double fov_max_v; //= 0.0558;
            double fov_max_v_d=1;
            double fov_max_v_i;
            double fov_center_d=1;
            double fov_center_i;
            double ring7_v=00.0557982, ring7_center_d=1, ring7_center_i;
            double ring6_v=-0.0000000, ring6_center_d=1, ring6_center_i;
            double ring5_v=-0.0557982, ring5_center_d=1, ring5_center_i;
            double ring4_v=-0.1110030, ring4_center_d=1, ring4_center_i;
            double ring3_v=-0.1651950, ring3_center_d=1, ring3_center_i;
            double ring2_v=-0.2180090, ring2_center_d=1, ring2_center_i;
            double ring1_v=-0.2692000, ring1_center_d=1, ring1_center_i;
            double ring0_v=-0.3185050, ring0_center_d=1, ring0_center_i;




            double fov_center6_v=00.0000000,fov_center6_d=1, fov_center6_i;
            double fov_center5_v=-0.0557982,fov_center5_d=1, fov_center5_i;
            double fov_center4_v=-0.1110030,fov_center4_d=1, fov_center4_i;
            double fov_center3_v=-0.1651950,fov_center3_d=1, fov_center3_i;
            double fov_center2_v=-0.2180090,fov_center2_d=1, fov_center2_i;
            double fov_center1_v=-0.2692100,fov_center1_d=1, fov_center1_i;
            double fov_center0_v=-0.3185050,fov_center0_d=1, fov_center0_i;

            //RING V ANGLES ARE CONSTANT
            double RING_7_v = 0.0557982;
            double RING_6_v = 0;
            double RING_5_v = -0.0557982;
            double RING_4_v = -0.111003;
            double RING_3_v = -0.165195;
            double RING_2_v = -0.218009;
            double RING_1_v = -0.2692;
            double RING_0_v = -0.318505;

            
                        


            //int min_age = 100000;

            // Recording exclusion zone and field of view
            int exclusion_points_count = 0;
            bool prev_point_differed = false;
            for (auto i = comp_scan.begin(); i != comp_scan.end(); ++i) {              

              /*if (i->second[0] == 0 && i->second[4] == 6) {
                std::cout << "VIEWING point - h:" << i->second[0] << ", v:" << i->second[1] << ", d:" << i->second[2] << ", i:" << i->second[3] << ", r:" << i->second[4] << ", t:" << i->second[5] << std::endl;
              }*/

              //std::cout << i->second[5] << ", ";
              //if (i->second[5] < min_age) {
              //  min_age = i->second[5];
              //}
              
              /*// Get fov min_h and min_v data
              if (i == comp_scan.begin()) {
                fov_min_h = i->second[0];
                fov_min_h_d = i->second[2];
                fov_min_h_i = i->second[3];

                fov_min_v = i->second[1];
                fov_min_v_d = i->second[2];
                fov_max_v_i = i->second[3];
              }

              // Get fov max_h and max_v data
              if (i == comp_scan.end()) {
                fov_max_h = i->second[0];
                fov_max_h_d = i->second[2];
                fov_max_h_i = i->second[3];

                fov_max_v = i->second[1];
                fov_max_v_d = i->second[2];
                fov_max_v_i = i->second[3];
              }*/

              // Get fov max_h data
              if (i->second[0]>fov_max_h) {
                fov_max_h = i->second[0];
                fov_max_h_d = i->second[2];
                fov_max_h_i = i->second[3];              
              }

              // Get fov min_h data
              if (i->second[0]<fov_min_h) {
                fov_min_h = i->second[0];
                fov_min_h_d = i->second[2];
                fov_min_h_i = i->second[3];
              }

              // Get ring 7 center point
              if (i->second[0] > -0.005 && i->second[0] < 0.005 && i->second[4] == 7) {
                ring7_center_d = i->second[2];
                ring7_center_i = i->second[3];
              }

              // Get ring 6 center point
              if (i->second[0] > -0.005 && i->second[0] < 0.005 && i->second[4] == 6) {
                ring6_center_d = i->second[2];
                ring6_center_i = i->second[3];
              }

              // Get ring 3 center point
              if (i->second[0] > -0.005 && i->second[0] < 0.005 && i->second[4] == 3) {
                //std::cout << "updating fov_center3 from " << fov_center3_d << " to " << i->second[2] << ", h:" << i->second[0] << std::endl;
                ring3_center_d = i->second[2];
                ring3_center_i = i->second[3];
              }

              // Get ring 0 center point
              if (i->second[0] > -0.005 && i->second[0] < 0.005 && i->second[4] == 0) {
                ring0_center_d = i->second[2];
                ring0_center_i = i->second[3];
              }

              // Get exclusion zone data              
              if (base_scan.find(i->first) != base_scan.end()) {                
                double distance = base_scan.find(i->first)->second[2];                  
                if (std::abs(i->second[2] - distance) > distance *0.05) {
                  if (i->second[5] < 2) {
                    i->second[5]++;
                  }

                  if (i->second[5] >= 2) {
                    //std::cout << "c distance:" << i->second[2] << ", b distance:" << distance << "h:" << i->second[0] << "v:" << i->second[1] << std::endl;
                    if (prev_point_differed == true) {
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
                      ++exclusion_points_count;
                    }                    
                    prev_point_differed = true;
                  }
                } else {                  
                  prev_point_differed = false;
                  if (i->second[5] > 0) {
                    i->second[5]--;
                  }
                }
                //std::cout << "found" << std::endl;
              }
              //std::cout << i->second[5] << ", ";              
            }
            //std::cout << "----------------------------------------------" << std::endl;
            //std::cout << std::endl;
            //std::cout << exclusion_points_count << std::endl;



            //std::cout << std::endl << std::endl;
            //std::cout << "MIN: " << min_age << "---------------------------" << std::endl << std::endl << std::endl;
            //std::cout << origin_distance << " " << fov_center7_d << " " << fov_center6_d << " " << fov_center3_d << " " << fov_center0_d << std::endl;
            

            //RING V ANGLES TRANSFORMED FOR SENSOR PROJECTOR OFFSET
            //THE OPTICAL ORIGIN OF THE SENSOR IS 0.195 METERS DIRECTLY ABOVE THE OPTICAL ORIGIN OF THE PROJECTOR
            double offset_y = 0.195; 
            ring7_v = (atan((offset_y + ring7_center_d * sin(ring7_v))/(ring7_center_d * cos(ring7_v))));
            ring6_v = (atan((offset_y + ring6_center_d * sin(ring6_v))/(ring6_center_d * cos(ring6_v))));
            ring5_v = (atan((offset_y + ring5_center_d * sin(ring5_v))/(ring5_center_d * cos(ring5_v))));
            ring4_v = (atan((offset_y + ring4_center_d * sin(ring4_v))/(ring4_center_d * cos(ring4_v))));
            ring3_v = (atan((offset_y + ring3_center_d * sin(ring3_v))/(ring3_center_d * cos(ring3_v))));
            ring2_v = (atan((offset_y + ring2_center_d * sin(ring2_v))/(ring2_center_d * cos(ring2_v))));
            ring1_v = (atan((offset_y + ring1_center_d * sin(ring1_v))/(ring1_center_d * cos(ring1_v))));
            ring0_v = (atan((offset_y + ring0_center_d * sin(ring0_v))/(ring0_center_d * cos(ring0_v))));

            //RING V ANGLES CONVERTED FROM RADIANS TO DEGREES
            ring7_v = ring7_v*180/3.141592;
            ring6_v = ring6_v*180/3.141592;
            ring5_v = ring5_v*180/3.141592;
            ring4_v = ring4_v*180/3.141592;
            ring3_v = ring3_v*180/3.141592;
            ring2_v = ring2_v*180/3.141592;
            ring1_v = ring1_v*180/3.141592;
            ring0_v = ring0_v*180/3.141592;

            std::cout << "ring7_v = " << ring7_v << ", ring7_center_d = " << ring7_center_d<< "ring0_v = " << ring0_v << ", ring0_center_d = " << ring0_center_d <<  std::endl;           

            // Switching max and min
            // Right of center is negative, left of center is positive
            //std::cout << "count: " << count << ", min_h " << min_h << ", max_h " << max_h << ", fov_min_h " <<fov_min_h<< ", fov_max_h " <<fov_max_h<< std::endl;
            auto temp = min_h;
            min_h = max_h*-1;
            max_h = temp*-1;
            temp = fov_min_h;
            fov_min_h = fov_max_h*-1;
            fov_max_h = temp*-1;
            fov_max_h = fov_max_h*180/3.141592;
            fov_min_h = fov_min_h*180/3.141592;


            //std::cout << "count: " << count << ", min_h " << min_h << ", max_h " << max_h << ", fov_min_h " <<fov_min_h<< ", fov_max_h " <<fov_max_h<< std::endl;

            // Margin setup
            // double margine_x = 0; // Margin width in meters
            // double margine_y = 0;
            // double max_vm = max_v+0.0567+atan(margine_y/max_v_distance); // add angle (0.0567 radians, 3.25 degrees) to protect the space before next empty ring and add margin angle in radians
            // double min_vm = min_v-0.0567-atan(margine_y/min_v_distance); //-0.0567


            // Sensor offset compensation (vertical offset only)
            //height of sensor origin above scanner origin in meters (measured 0.195 +/-0.002)
            // double max_vt = atan((offset_y+max_v_distance*sin(max_vm))/(max_v_distance*cos(max_vm))); //v transformed to projector origin
            // double min_vt = atan((offset_y+min_v_distance*sin(min_vm))/(min_v_distance*cos(min_vm)));
            double max_yc = 191*(atan((offset_y+max_v_distance*sin(max_v))/(max_v_distance*cos(max_v)))); 
            // double min_yc = 191*(atan((offset_y+min_v_distance*sin(min_v))/(min_v_distance*cos(min_v))));
            double max_xc = max_h*191;
            // double min_xc = min_h*191;
            // double avg_xc = min_xc+((max_xc-min_xc)/2);
            // double avg_yc = min_yc+((max_yc-min_yc)/2);
            
            // Scale for beyond software
            // Projector field of view is roughly -30 t0 +30 degrees
            // Beyond software window clipout effect value range is -100 to 100
            // x = (h+margine)*(180\PI)*(100\30)
            // double max_x = (max_h+margine_x)*191;
            // double min_x = (min_h-margine_x)*191;
            // double min_y;
            // if (min_v < -0.3) {min_y = -110;}
            // if (min_v > -0.3) {min_y = min_vt*191;} 
            // double max_y;
            // if (max_v > 0.04) {max_y = 110;}
            // if (max_v < 0.04) {max_y = max_vt*191;} 

            //std::cout << "count: " << count << ", fov_max_v: " << fov_max_v << ", fov_min_v: " << fov_min_v<< ", fov_max_v_distance: " << fov_max_v_distance << ", fov_min_v_distance: " << fov_min_v_distance << std::endl;
            //std::cout << "count: " << count << ", fov_centter_d " << fov_center_d << ", max_v " << max_v << std::endl;
            
            //DETERMINE INTERCEPT ANGLES
            //Intercept angle = arctangent(distance2*sin(theta)/distance1-distance2*cos(theta))
            double intercept_h;
            double intercept_v;
            intercept_v=atan((fov_center4_d*sin(-RING_4_v))/(fov_center6_d - fov_center4_d*cos(-RING_4_v)))*(180/3.141592);
              if (intercept_v < 0) {intercept_v=180+intercept_v;}
            //std::cout << "intercept_v: " << intercept_v << "fov_center6_d: " << fov_center6_d << "fov_center4_d: " << fov_center4_d <<std::endl;

            UdpTransmitSocket transmitSocket( IpEndpointName( ADDRESS, SENDING_PORT ) );
            char buffer[OUTPUT_BUFFER_SIZE];
            osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
            
            p << osc::BeginBundleImmediate
                << osc::BeginMessage("/b/IPS.LIDAR_SCAN_STATUS.Caption") << ("CONNECTED") 
                << osc::EndMessage
                //PROTECT 1
                // << osc::BeginMessage("/b/IPS/PROTECT_1.Effect.0/Keys.0/Value1") << (float)(min_x)
                // << osc::EndMessage
                // << osc::BeginMessage("/b/IPS/PROTECT_1.Effect.0/Keys.0/Value2") << (float)(max_x)
                // << osc::EndMessage
                // << osc::BeginMessage("/b/IPS/PROTECT_1.Effect.0/Keys.0/Value3") << (float)(min_y)
                // << osc::EndMessage
                // << osc::BeginMessage("/b/IPS/PROTECT_1.Effect.0/Keys.0/Value4") << (float)(max_y)
                // << osc::EndMessage
                
                // SEND FIELD OF VIEW DATA
                << osc::BeginMessage("/b/LIDARDATA.FOV_MIN_H.Value") << (float)(fov_min_h)
                << osc::EndMessage                
                << osc::BeginMessage("/b/LIDARDATA.FOV_MAX_H.Value") << (float)(fov_max_h)
                << osc::EndMessage
                << osc::BeginMessage("/b/LIDARDATA.FOV_MIN_V.Value") << (float)(ring0_v)
                << osc::EndMessage                
                << osc::BeginMessage("/b/LIDARDATA.FOV_MAX_V.Value") << (float)(ring7_v)
                << osc::EndMessage
                

                // SEND RING CENTER ANGLE DATA
                << osc::BeginMessage("/b/VARIABLES.ring7_v") << (float)(ring7_v)
                << osc::EndMessage
                << osc::BeginMessage("/b/VARIABLES.ring6_v") << (float)(ring6_v)
                << osc::EndMessage
                << osc::BeginMessage("/b/VARIABLES.ring3_v") << (float)(ring3_v)
                << osc::EndMessage
                << osc::BeginMessage("/b/VARIABLES.ring0_v") << (float)(ring0_v)
                << osc::EndMessage

                
                // SEND RING CENTER DISTANCE DATA
                << osc::BeginMessage("/b/VARIABLES.ring7_center_d") << (float)(ring7_center_d)
                << osc::EndMessage
                << osc::BeginMessage("/b/VARIABLES.ring6_center_d") << (float)(ring6_center_d)
                << osc::EndMessage
                << osc::BeginMessage("/b/VARIABLES.ring3_center_d") << (float)(ring3_center_d)
                << osc::EndMessage
                << osc::BeginMessage("/b/VARIABLES.ring0_center_d") << (float)(ring0_center_d)
                << osc::EndMessage
              
                
                // SEND RING CENTER INTENSITY DATA
                //<< osc::BeginMessage("/b/VARIABLES.fov_center7_i") << (float)(fov_center7_i)
                //<< osc::EndMessage                
                // << osc::BeginMessage("/b/VARIABLES.fov_center6_i") << (float)(fov_center6_i)
                // << osc::EndMessage              
                //<< osc::BeginMessage("/b/VARIABLES.fov_center3_i") << (float)(fov_center3_i)
                //<< osc::EndMessage                
                // << osc::BeginMessage("/b/VARIABLES.fov_center0_i") << (float)(fov_center0_i)
                // << osc::EndMessage

                //SEND CALIBRATION DATA
                << osc::BeginMessage("/b/VARIABLES.intercept_v") << (float)(intercept_v)
                << osc::EndMessage
                << osc::BeginMessage("/b/CALIBRATIONDYNAMIC.PositionX") << (int)(max_xc)
                << osc::EndMessage
                << osc::BeginMessage("/b/CALIBRATIONDYNAMIC.PositionY") << (int)(max_yc)
                << osc::EndMessage
                << osc::BeginMessage("/b/VARIABLES.origin_baseline_distance") << (origin_baseline_distance)
                << osc::EndMessage
                << osc::BeginMessage("/b/VARIABLES.origin_baseline_intensity") << (origin_baseline_intensity)
                << osc::EndMessage
                << osc::BeginMessage("/b/VARIABLES.origin_distance") << (origin_distance)
                << osc::EndMessage
                << osc::BeginMessage("/b/VARIABLES.origin_intensity") << (origin_intensity)
                << osc::EndMessage             

                // Exclusion zone data                
                
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

            << osc::EndBundle;
                
            
            transmitSocket.Send( p.Data(), p.Size() );
          }

          visits_here++;

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
