#ifndef LOLA_POSE_FILE_SERVICE_H__
#define LOLA_POSE_FILE_SERVICE_H__

#include "lola/PoseService.h"
#include <atomic>
#include <vector>

/**
 * This allows to replay pose data saved before by the recorder module
 */
class PoseFileService : public PoseService {
public:
  /**
   * Create a new `PoseService` that will take the given file
   * and replay the saved pose data
   */
  PoseFileService(std::string const& filename);

  /**
  * Obtains the current pose information. Using this method is completely
  * thread safe.
  */
  HR_Pose_Red getCurrentPose() const override;

  /**
   * Dispatches the next frame
   */
  void triggerNextFrame() override;

private:

  /**
   * The pose data read from the file
   */
  std::vector<HR_Pose_Red> pose_data_;

  /**
   * The current pose in the vector
   */
  std::atomic<size_t> pose_idx_;

};

#endif
