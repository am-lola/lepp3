#ifndef LOLA_VISION_MESSAGE_H__
#define LOLA_VISION_MESSAGE_H__

// #include "deps/easylogging++.h"

// The macro creates an ID for a Robot message.
// The macro is taken from the LOLA source base.
// TODO Once C++11 can be used, make this a `constexpr` function, instead of a macro.
#define __MSG_ID_DEF_GLOBAL(dom,sig)  (0x80000000 | ((dom&0xFF)<<0x10) | (sig&0xFFFF))


enum Obstacle_Action
{
  SET_SSV = 0,
  MODIFY_SSV,
  REMOVE_SSV_DEL_WHOLE_SEGMENT_FLAG,
  REMOVE_SSV_DEL_ONLY_PART_FLAG
};

enum Message_Type
{
  Obstacle,
  Surface,
  RGB_Image,
  PointCloud
};

struct ObstacleMessage {
  uint32_t type;    // sphere, capsule, etc
  uint32_t model_id;
  uint32_t part_id;
  Obstacle_Action action; // add, remove, modify, ...
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
      msg.type = type_id;
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
    msg.type = type_id;
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
  uint32_t type;  // what type of content is contained in the message
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
    out << "[" << msg.header.type << " | " << msg.header.len << "|";
    out << "]";

    return out;
  }

};




#endif // LOLA_VISION_MESSAGE_H__