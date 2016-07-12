#ifndef LOLA_VISION_MESSAGE_H__
#define LOLA_VISION_MESSAGE_H__

// #include "deps/easylogging++.h"

// The macro creates an ID for a Robot message.
// The macro is taken from the LOLA source base.
// TODO Once C++11 can be used, make this a `constexpr` function, instead of a macro.
#define __MSG_ID_DEF_GLOBAL(dom,sig)  (0x80000000 | ((dom&0xFF)<<0x10) | (sig&0xFFFF))

  // IDs for particular vision operations.
  static uint32_t const SET_SSV    =  __MSG_ID_DEF_GLOBAL(0x4, 0x203);
  static uint32_t const MODIFY_SSV =  __MSG_ID_DEF_GLOBAL(0x4, 0x206);
  static uint32_t const REMOVE_SSV_DEL_WHOLE_SEGMENT_FLAG =  __MSG_ID_DEF_GLOBAL(0x4, 0x207);
  static uint32_t const REMOVE_SSV_DEL_ONLY_PART_FLAG     =  __MSG_ID_DEF_GLOBAL(0x4, 0x209); // TODO: MAKE SURE THIS IS OK!


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
    std::cout << "Constructing delete full message with object_id = " << model_id;
    ObstacleMessage msg;

    msg.action = REMOVE_SSV_DEL_WHOLE_SEGMENT_FLAG;
    msg.model_id = model_id;

    return msg;
  }

  /**
   * Creates a `ObstacleMessage` that says that a particular part of a larger model
   * should be deleted.
   */
  static ObstacleMessage DeletePartMessage(int model_id, int part_id)
  {
      std::cout << "Constructing delete part message with (model_id, part_id) = (" << model_id << ", " << part_id << ")";

      ObstacleMessage msg;
      msg.model_id = model_id;
      msg.part_id = part_id;
      msg.action = REMOVE_SSV_DEL_ONLY_PART_FLAG;

      return msg;
  }
  /**
   * Creates a `ObstacleMessage` that says that a new object with the given
   * parameters should be created.
   */
  static ObstacleMessage SetMessage(
      int type_id, int model_id, int part_id, double radius, std::vector<double> const& coefs)
  {
      std::cout << "Constructing set message with (model_id, part_id) = (" << model_id << ", " << part_id << ")";
      ObstacleMessage msg;
      msg.action = SET_SSV;
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
    std::cout << "Constructing set message with (model_id, part_id) = (" << model_id << ", " << part_id << ")";
    
    ObstacleMessage msg;
    msg.action = MODIFY_SSV;
    msg.type = (ObstacleType)type_id;
    msg.model_id = model_id;
    msg.part_id = part_id;
    msg.radius = radius;

    memset(msg.coeffs, 0, sizeof msg.coeffs);

    for (size_t i = 0; i < coefs.size(); ++i) msg.coeffs[i] = coefs[i];

    return msg;
  }


};


struct SurfaceMessage {
  uint32_t id;
  uint32_t action; // add, remove, modify, ...
  float normal[3];
  float vertices[24];
};

struct RGBMessage {
  uint32_t height;
  uint32_t width;
  char* pixels;
};

struct PointCloudMessage {
  uint32_t format; // colored, etc.
  char* data;
};

/**
 * A struct representing the header of all messages sent from the vision node
 */
struct VisionMessageHeader {
  Message_Type type;  // what type of content is contained in the message
  uint32_t len;   // size of content
  uint32_t frame; // frame_id the information belongs to
};

/**
 * A struct representing the raw vision message format that is sent to the
 * robot.
 */
struct VisionMessage {
  VisionMessageHeader header;
  const char* content;

  VisionMessage(Message_Type type, ObstacleMessage msg)
  {
    header.type = type;
    header.len = sizeof(msg);
    content = (char const*)(&msg);
  }

  VisionMessage(Message_Type type, SurfaceMessage msg)
  {
    header.type = type;
    header.len = sizeof(msg);
    content = (char const*)(&msg);
  }

  friend std::ostream& operator<<(std::ostream& out, VisionMessage const& msg)
  {
    out << "[" << (msg.header.type == Message_Type::Obstacle ? "Obstacle" : "Surface")  << " | " << msg.header.len << " bytes]]\n";

    switch (msg.header.type)
    {
      case Message_Type::Obstacle:
      {
        ObstacleMessage* msgContent = (ObstacleMessage*)(msg.content);
        out << "\ttype: " << (msgContent->type == ObstacleType::Sphere ? "Sphere" : "Capsule")  << ", id: " << msgContent->model_id << "|" << msgContent->part_id << "\n";
        out << "\taction: 0x" << std::hex << msgContent->action << std::dec << ", radius: " << msgContent->radius << "\n";
        out << "\tModel Coefficients: [";
        for (int i = 0; i < 9; i++)
        {
          out << msgContent->coeffs[i];
          if (i < 8)
            out << ", ";
        }
        out << "]";
        break;
      }
      case Message_Type::Surface:
      {
        out << "Some kind of surface";
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




#endif // LOLA_VISION_MESSAGE_H__