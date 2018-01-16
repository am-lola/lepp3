#ifndef LOLA_VISION_MESSAGE_H__
#define LOLA_VISION_MESSAGE_H__

#include <string.h>
#include <iface_sig_wpatt.hpp>
#include <vector>
#include <iostream>


#pragma pack(push,1)

namespace am2b_iface
{
    enum Message_Type
    {
      Obstacle = 0,
      Surface,
      RGB_Image,
      PointCloud
    };

    enum ObstacleType
    {
      Sphere = 0,
      Capsule,
      Triangle
    };

    struct ObstacleMessage {
      ObstacleType type;    // sphere, capsule, etc
      uint32_t model_id;
      uint32_t part_id;
      uint32_t action; // add, remove, modify, ...
      float radius;
      int32_t surface = -1;
      float coeffs[9];

      // Static factory functions. Facilitate creating the messages without worrying
      // about the internal format.
      /**
       * Creates a `ObstacleMessage` that says that an object with the given ID
       * should be removed. The entire model is removed, including all of its child
       * parts.
       */
      static ObstacleMessage DeleteMessage(int model_id)
      {
        // std::cout << "Constructing delete full message with object_id = " << model_id << std::endl;
        ObstacleMessage msg;

        msg.action = am2b_iface::REMOVE_SSV_WHOLE_SEGMENT;
        msg.model_id = model_id;

        return msg;
      }

      /**
       * Creates a `ObstacleMessage` that says that a particular part of a larger model
       * should be deleted.
       */
      static ObstacleMessage DeletePartMessage(int model_id, int part_id)
      {
          // std::cout << "Constructing delete part message with (model_id, part_id) = ("
	  //           << model_id << ", " << part_id << ")" << std::endl;;

          ObstacleMessage msg;
          msg.model_id = model_id;
          msg.part_id = part_id;
          msg.action = am2b_iface::REMOVE_SSV_ONLY_PART;

          return msg;
      }
      /**
       * Creates a `ObstacleMessage` that says that a new object with the given
       * parameters should be created.
       */
      static ObstacleMessage SetMessage(int type_id, int model_id, int part_id, double radius,
					std::vector<double> const& coefs)
      {
          // std::cout << "Constructing set message with (model_id, part_id) = ("
	  //           << model_id << ", " << part_id << ")" << std::endl;
          ObstacleMessage msg;
          msg.action = am2b_iface::SET_SSV;
          msg.type = (ObstacleType)type_id;
          msg.model_id = model_id;
          msg.part_id = part_id;
          msg.radius = radius;

          memset(msg.coeffs, 0, sizeof msg.coeffs);

          for (size_t i = 0; i < coefs.size(); ++i) msg.coeffs[i] = coefs[i];

          return msg;
      }
      /**
       * Creates a `ObstacleMessage` that says that an existing object with the given
       * ID should be modified according to the given parameters.
       */
      static ObstacleMessage ModifyMessage(
          int type_id, int model_id, int part_id, double radius, std::vector<double> const& coefs)
      {
        // std::cout << "Constructing set message with (model_id, part_id) = ("
	// 	  << model_id << ", " << part_id << ")" << std::endl;;

        ObstacleMessage msg;
        msg.action = am2b_iface::MODIFY_SSV;
        msg.type = (ObstacleType)type_id;
        msg.model_id = model_id;
        msg.part_id = part_id;
        msg.radius = radius;

        memset(msg.coeffs, 0, sizeof msg.coeffs);

        for (size_t i = 0; i < coefs.size(); ++i) msg.coeffs[i] = coefs[i];

        return msg;
      }
      /**
       * Creates a `ObstacleMessage` that says that a new object with the given
       * parameters should be created.
       */
      static ObstacleMessage SetMessage(int type_id, int model_id, int part_id, double radius,
					std::vector< std::vector<float> > const& coefs)
      {
	// std::cout << "Constructing set message with (model_id, part_id) = ("
	// 	  << model_id << ", " << part_id << ")" << std::endl;;
	ObstacleMessage msg;
	msg.action = am2b_iface::SET_SSV;
	msg.type = (ObstacleType)type_id;
	msg.model_id = model_id;
	msg.part_id = part_id;
	msg.radius = radius;

	memset(msg.coeffs, 0, sizeof msg.coeffs);

	int counter = 0;
	if(coefs.size() != 3)
          std::cout<<"coefs size not correct! We assume always three corner points!"<<std::endl;

	for (size_t ii = 0; ii<coefs.size();++ii)
	  {
	    for(size_t jj = 0; jj<coefs[ii].size(); ++jj)
	      {
		msg.coeffs[counter] = coefs[ii][jj];
		++counter;
	      }
	  }

	return msg;
      }
      /**
       * Creates a `ObstacleMessage` that says that an existing object with the given
       * ID should be modified according to the given parameters.
       */
      static ObstacleMessage ModifyMessage(
          int type_id, int model_id, int part_id, double radius,
	  std::vector< std::vector<double> > const& coefs)
      {
        std::cout << "Constructing set message with (model_id, part_id) = ("
		  << model_id << ", " << part_id << ")" << std::endl;;

        ObstacleMessage msg;
        msg.action = am2b_iface::MODIFY_SSV;
        msg.type = (ObstacleType)type_id;
        msg.model_id = model_id;
        msg.part_id = part_id;
        msg.radius = radius;

        memset(msg.coeffs, 0, sizeof msg.coeffs);

	int counter = 0;
	if(coefs.size() != 3)
          std::cout<<"coefs size not correct! We assume always three corner points!"<<std::endl;

	for (size_t ii = 0; ii<coefs.size();++ii)
	  {
	    for(size_t jj = 0; jj<coefs[ii].size(); ++jj)
	      {
		msg.coeffs[counter] = coefs[ii][jj];
		++counter;
	      }
	  }

        return msg;
      }



      // << overload to make printing message content easier
      friend std::ostream& operator<<(std::ostream& out, ObstacleMessage const& msg)
      {

        out << "\ttype: " << (msg.type == ObstacleType::Sphere ? "Sphere" : msg.type == ObstacleType::Capsule ? "Capsule" : "Triangle")
	    << ", id: " << msg.model_id << "|"
	    << msg.part_id << "\n";
        out << "\taction: 0x" << std::hex << msg.action << std::dec
	    << ", radius: " << msg.radius
	    << ", surface: " << msg.surface << "\n";
        out << "\tModel Coefficients: [";
        for (int i = 0; i < 9; i++)
        {
          out << msg.coeffs[i];
          if (i < 8)
            out << ", ";
        }
        out << "]";

        return out;
      }
    };


    struct SurfaceMessage {
      uint32_t id;
      uint32_t action; // add, remove, modify, ...
      float normal[3];
      float vertices[24];

      static SurfaceMessage SetMessage( int id,
          std::vector<float> normal,
          std::vector<float> vertices)
      {
        SurfaceMessage msg;
        msg.id = id;
        msg.action = am2b_iface::SET_SURFACE;
        for (size_t i = 0; i < 3; i++)
        {
          msg.normal[i] = normal[i];
        }
                          //  8 points w/ 3 coordinates each
        for (size_t i = 0; i < 3 * 8; i++)
        {
          msg.vertices[i] = vertices[i];
        }

        return msg;
      }

      static SurfaceMessage SetMessage( int id,
          std::vector<float> const& normal,
          std::vector< std::vector<float> > const& vertices)
      {
        SurfaceMessage msg;
        msg.id = id;
        msg.action = am2b_iface::SET_SURFACE;
        for (size_t i = 0; i < 3; i++)
        {
          msg.normal[i] = normal[i];
        }
                          //  8 points w/ 3 coordinates each
        int counter = 0;
        if(vertices.size() == 0)
	  {
            std::cout<<"Vertices size equals zero? Returning default msg!\n"
		     <<std::endl;
            return msg;
          }
        if(vertices.size() != 8)
	  {
	    std::cout<<"Vertices size not correct! We assume always eight corner points!\n"
		     <<"Fill up with last point!\n"
		     <<std::endl;
	  }
        for (size_t ii = 0; ii<vertices.size();++ii)
        {
          for(size_t jj = 0; jj<vertices[ii].size(); ++jj)
          {
            msg.vertices[counter] = vertices[ii][jj];
            ++counter;
          }
        }

	// fill up with last corner point
        for (size_t ii = vertices.size(); ii<8;++ii)
        {
          for(size_t jj = 0; jj<vertices[vertices.size()-1].size(); ++jj)
          {
            msg.vertices[counter] = vertices[vertices.size()-1][jj];
            ++counter;
          }
        }

        return msg;
      }

      static SurfaceMessage ModifyMessage(int id,
            std::vector<float> normal,
            std::vector<float> vertices)
      {
        SurfaceMessage msg;
        msg.id = id;
        msg.action = am2b_iface::MODIFY_SURFACE;
        for (size_t i = 0; i < 3; i++)
        {
          msg.normal[i] = normal[i];
        }
                          //  8 points w/ 3 coordinates each
        for (size_t i = 0; i < 3 * 8; i++)
        {
          msg.vertices[i] = vertices[i];
        }

        return msg;
      }

      static SurfaceMessage ModifyMessage(int id,
            std::vector<float> const& normal,
            std::vector< std::vector<float> > const& vertices)
      {
        SurfaceMessage msg;
        msg.id = id;
        msg.action = am2b_iface::MODIFY_SURFACE;
        for (size_t i = 0; i < 3; i++)
        {
          msg.normal[i] = normal[i];
        }
                          //  8 points w/ 3 coordinates each
        int counter = 0;
        if(vertices.size() != 8)
	  {
                std::cout<<"Vertices size not correct! We assume always eight corner points!\n"
			 <<"Fill up with last point!\n"
			 <<std::endl;
	  }
        for (size_t ii = 0; ii<vertices.size();++ii)
        {
          for(size_t jj = 0; jj<vertices[ii].size(); ++jj)
          {
            msg.vertices[counter] = vertices[ii][jj];
            ++counter;
          }
        }

	// fill up with last corner point
        for (size_t ii = vertices.size(); ii<8;++ii)
        {
          for(size_t jj = 0; jj<vertices[vertices.size()-1].size(); ++jj)
          {
            msg.vertices[counter] = vertices[vertices.size()-1][jj];
            ++counter;
          }
        }


        return msg;
      }


      static SurfaceMessage DeleteMessage(int id)
      {
        SurfaceMessage msg;
        msg.id = id;
        msg.action = am2b_iface::REMOVE_SURFACE;
        return msg;
      }

            // << overload to make printing message content easier
      friend std::ostream& operator<<(std::ostream& out, SurfaceMessage const& msg)
      {

        out << "\tid: " << msg.id;
        out << "\taction: 0x" << std::hex << msg.action<<"\n";
        out << "\tnormal: [";
        for (int i = 0; i < 3; i++)
        {
          out << msg.normal[i];
          if (i < 3-1)
            out << ", ";
        }
        out << "]\n";
        out << "\tvertices: [";
        for (int i = 0; i < 24; i++)
        {
          out << msg.vertices[i];
          if (i < 24-1)
            out << ", ";
        }
        out << "]\n";

        return out;
      }

    };

    struct RGBMessage {
      uint32_t height;
      uint32_t width;
      const unsigned char* pixels;

      RGBMessage(const unsigned char* rgb_data, uint32_t img_width, uint32_t img_height)
      {
        height = img_height;
        width = img_width;
        pixels = rgb_data;
      }
    };

    struct PointCloudMessage {
      uint32_t format; // colored, etc.
      uint32_t count;  // number of points
      unsigned char* data;

      PointCloudMessage(unsigned char* pt_data, uint32_t numpoints, uint32_t pointsize)
      {
        format = pointsize;
        count = numpoints;
        data = new unsigned char[numpoints*pointsize];
        memcpy(data, pt_data, numpoints*pointsize);
      }
    };

    /**
     * A struct representing the header of all messages sent from the vision node
     */
    struct VisionMessageHeader {
      Message_Type type;  // what type of content is contained in the message
      uint32_t len;   // size of content
      uint32_t frame; // frame_id the information belongs to

      // << overload to make printing header details easier
      friend std::ostream& operator<<(std::ostream& out, VisionMessageHeader const& header)
      {
        out << "[";
        switch(header.type)
        {
          case Message_Type::Obstacle:
          {
            out << "Obstacle";
            break;
          }
          case Message_Type::Surface:
          {
            out << "Surface";
            break;
          }
          case Message_Type::PointCloud:
          {
            out << "PointCloud";
            break;
          }
          case Message_Type::RGB_Image:
          {
            out << "RGB Image";
            break;
          }
          default:
          {
            out << "Unkown Message Type";
            break;
          }
        }
        out  << " | " << "frame #" << header.frame << " | " << header.len << " bytes]";
        return out;
      }
    };

    /**
     * A struct representing the raw vision message format that is sent to the
     * robot.
     */
    struct VisionMessage {
      /*
        When VisionMessages are sent over the network, only the header and the data pointed
        to by content are sent. We don't send the whole VisionMessage struct (which includes
        a pointer that would not be useful on the other side).
       */
      VisionMessageHeader header;
      char* content = nullptr;

      VisionMessage(const ObstacleMessage& msg, long frame_num)
      {
        header.type = Message_Type::Obstacle;
        header.len = sizeof(msg);
        header.frame = frame_num;
        content = new char[sizeof(msg)];

        // make our own copy of the message
        memcpy(content, &msg, sizeof(msg));

        std::cout<<*this<<std::endl;
      }

      VisionMessage(const SurfaceMessage& msg, long frame_num)
      {
        header.type = Message_Type::Surface;
        header.len = sizeof(msg);
        header.frame = frame_num;
        content = new char[sizeof(msg)];

        // make our own copy of the message
        memcpy(content, &msg, sizeof(msg));

        std::cout<<*this<<std::endl;
      }

      VisionMessage(const PointCloudMessage& msg, long frame_num)
      {
        header.type = Message_Type::PointCloud;
        header.len = sizeof(msg) + msg.format * msg.count;
        header.frame = frame_num;
        content = new char[header.len];
        // copy message details
        memcpy(content, &msg, sizeof(msg));
        // copy point data to end of buffer
        memcpy(content + sizeof(msg), msg.data, msg.format * msg.count);
      }

      VisionMessage(const RGBMessage& msg, long frame_num)
      {
        header.type = Message_Type::RGB_Image;
        header.len = sizeof(msg) + 3 * msg.height * msg.width; // 1 byte per color channel per pixel
        header.frame = frame_num;
        content = new char[header.len];
        // copy message
        memcpy(content, &msg, sizeof(msg));
        // copy image data to end of buffer
        memcpy(content + sizeof(msg), msg.pixels, 3 * msg.height * msg.width);
      }

      // copy constructor
      VisionMessage(const VisionMessage& msg) : header(msg.header)
      {
        content = new char[header.len];
        memcpy(content, msg.content, header.len);
      }

      ~VisionMessage()
      {
        if (content != nullptr)
        {
          delete[] content;
          content = nullptr;
        }
      }

      // << overload to make printing message details easier
      friend std::ostream& operator<<(std::ostream& out, VisionMessage const& msg)
      {
        out << msg.header << std::endl;

        switch (msg.header.type)
        {
          case Message_Type::Obstacle:
          {
            // ObstacleMessage* msgContent = (ObstacleMessage*)(msg.content);
            // out << *msgContent;
            break;
          }
          case Message_Type::Surface:
	    {
	      // SurfaceMessage* msgContent = (SurfaceMessage*)(msg.content);
	      // out << *msgContent;
	      break;
          }
          case Message_Type::PointCloud:
          {
            out << "PointCloud";
            break;
          }
          case Message_Type::RGB_Image:
          {
            out << "RGB Image";
            break;
          }
          default:
          {
            out << "Unknown message type D:";
            break;
          }
        }

        return out;
      }

    };
}

#pragma pack(pop)

#endif // LOLA_VISION_MESSAGE_H__
