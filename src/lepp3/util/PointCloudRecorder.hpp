#ifndef LEPP3_RECORDER_HPP
#define LEPP3_RECORDER_HPP

#include "lepp3/Typedefs.hpp"
#include <am2b-arvis/ARVisualizer.hpp>
#include <fstream>

namespace lepp {

#define RECORDINGS_DIR "../recs/" // don't forget to add a trailing slash
#define USE_FRAMETIME

/**
 * Quick&dirty point cloud recording and playback for testing and repeatable debugging purposes.
 *
 * To use this for simple point cloud recordings:
 * 1. call reset()
 * 2. add new frames with newFrame( pointCloud )
 * 3. call saveToFile("path/to/file") to save the recorded point clouds to a file
 *
 * To play the recordings:
 * 1. attach as an observer
 * 2. use the UI to play the files (set RECORDINGS_DIR to the location of your recordings)
 */
class PointCloudRecorder : public FrameDataSubject, public FrameDataObserver {

  struct Frame {

    size_t startIdx;
    size_t numPoints;
#ifdef USE_FRAMETIME
    double frameTime;
#endif
  };

  struct Point {

    Point() = default;
    Point(float x_, float y_, float z_) : x(x_), y(y_), z(z_), w(1.0f) { }

    float x, y, z, w;
  };

public:

  PointCloudRecorder(ar::ARVisualizer* visualizer) {
    // Recording window
    _windowRecording = visualizer->AddUIWindow("Recording");
    _inputTextRecordingFile = _windowRecording->AddInputText("File");
#ifndef USE_FRAMETIME
    _sliderRecordingFrameTime = _windowRecording->AddDragInt("Frametime (ms)", 10, 10000, 1.0f, 33);
#endif
    _buttonPlayRecording = _windowRecording->AddButton("Play recording");
    _windowRecording->SameLine();
    _buttonPauseRecording = _windowRecording->AddButton("Toggle Pause");

    _textCurrentFrame = _windowRecording->AddText("Frame %d", 0);
    _buttonPrevFrame = _windowRecording->AddButton("<");
    _windowRecording->SameLine();
    _buttonNextFrame = _windowRecording->AddButton(">");

    _sliderBreakpoint = _windowRecording->AddDragInt("Breakpoint", 0, 10000, 1.0f);

    _windowRecording->AddSeparator();
    _buttonStartRecording = _windowRecording->AddButton("Start recording");
    _buttonStopRecording = _windowRecording->AddButton("Stop recording");

    uiThread_ = std::thread([this]() {
      while (true) {
        if (_windowRecording->GetButtonState(_buttonPlayRecording)) {

          playingRecording_ = false;

          if (playbackThread_.joinable())
            playbackThread_.join();

          playbackThread_ = std::thread([this]() {
            const std::string filePath = std::string(RECORDINGS_DIR) + _windowRecording->GetInputTextValue(_inputTextRecordingFile);
            this->playFile(filePath);
          });
        }

        if (_windowRecording->GetButtonState(_buttonPauseRecording)) {
          pausedRecording_ = !pausedRecording_;
        }

        if (_windowRecording->GetButtonState(_buttonNextFrame) && pausedRecording_) {
          currentFrame_++;
        }
        if (_windowRecording->GetButtonState(_buttonPrevFrame) && pausedRecording_) {
          currentFrame_--;
        }

        breakpointFrame_ = _windowRecording->GetSliderIntValue(_sliderBreakpoint);
      }
    });
  }

  void reset() {
    frames_.clear();
    points_.clear();
  }

  virtual void updateFrame(FrameDataPtr frameData) {
    if (recording_) {
      if (_windowRecording->GetButtonState(_buttonStopRecording))
      {
        std::time_t time = std::time(nullptr);
        char fileName[128];
        std::strftime(fileName, 128, "%c", std::localtime(&time));
        std::cout << "saving recording to file " << fileName << "\n";
        saveToFile(fileName);
        recording_ = false;
        return;
      }
      newFrame(frameData->cloudMinusSurfaces);
    } else {
      if (_windowRecording->GetButtonState(_buttonStartRecording))
      {
        recording_ = true;
        newFrame(frameData->cloudMinusSurfaces);
      }
    }
  }

  void newFrame(PointCloudConstPtr pointCloud) {

    const auto numPoints = pointCloud->size();
    const PointT* data = pointCloud->points.data();

    Frame frame;
    frame.startIdx = points_.size();
    frame.numPoints = numPoints;
#ifdef USE_FRAMETIME
    if (frames_.size() > 0) {
      _frameTimer.stop();
      frame.frameTime = _frameTimer.duration();
    } else {
      frame.frameTime = 0;
    }
#endif

    frames_.push_back(frame);

    points_.reserve(points_.size() + numPoints);

    for (size_t i = 0; i < numPoints; i++) {
      points_.emplace_back(data[i].x, data[i].y, data[i].z);
    }

    _frameTimer.start();
  }

  void saveToFile(const std::string& filePath) {

    using namespace std;

    ofstream fout(filePath);
    if (!fout.is_open())
      return;

    fout << frames_.size() << " " << points_.size() << endl;
    for (const Frame& frame : frames_) {
#ifdef USE_FRAMETIME
      fout << frame.startIdx << " " << frame.numPoints << " " << frame.frameTime << endl;
#else
      fout << frame.startIdx << " " << frame.numPoints << endl;
#endif
    }

    for (const Point& point : points_) {
      fout << point.x << " " << point.y << " " << point.z << endl;
    }

    fout.close();
  }

  void loadFromFile(const std::string& filePath) {

    std::ifstream fin(filePath);
    if (!fin.is_open())
      return;

    reset();

    size_t numFrames, numPoints;
    fin >> numFrames >> numPoints;
    frames_.resize(numFrames);
    points_.resize(numPoints);

    for (size_t i = 0; i < numFrames; i++) {
#ifdef USE_FRAMETIME
      fin >> frames_[i].startIdx >> frames_[i].numPoints >> frames_[i].frameTime;
#else
      fin >> frames_[i].startIdx >> frames_[i].numPoints;
#endif
    }

    for (size_t i = 0; i < numPoints; i++) {
      fin >> points_[i].x >> points_[i].y >> points_[i].z;
    }
  }

  void playFile(const std::string& filePath) {

    if (filePath != currentFile_)
      loadFromFile(filePath);

    currentFile_ = filePath;

    playbackRecording();
  }

  void getFrame(int frameNr, size_t& outNumPoints, PointT** outPoints) {

    const Frame& frame = frames_[frameNr];

    outNumPoints = frame.numPoints;
    *outPoints = (PointT*)(points_.data() + frame.startIdx);
  }

  inline int getNumFrames() const {
    return (int)frames_.size();
  }

  void pause() {
    pausedRecording_ = true;
  }

private:

  void playbackRecording();

  Vector<Point> points_;
  Vector<Frame> frames_;

  ar::IUIWindow* _windowRecording;
  ar::ui_element_handle _inputTextRecordingFile;
  ar::ui_element_handle _sliderRecordingFrameTime;
  ar::ui_element_handle _buttonPlayRecording;
  ar::ui_element_handle _buttonPauseRecording;
  ar::ui_element_handle _textCurrentFrame;
  ar::ui_element_handle _buttonNextFrame;
  ar::ui_element_handle _buttonPrevFrame;
  ar::ui_element_handle _sliderBreakpoint;

  ar::ui_element_handle _buttonStartRecording;
  ar::ui_element_handle _buttonStopRecording;

  Timer _frameTimer;

  std::string currentFile_;

  std::atomic_bool playingRecording_ { false };
  std::atomic_bool pausedRecording_ { false };
  std::atomic_bool recording_ { false };
  std::atomic<int> currentFrame_;
  std::atomic<int> breakpointFrame_;

  std::thread uiThread_;
  std::thread playbackThread_;
};

void PointCloudRecorder::playbackRecording() {

  playingRecording_ = true;
  pausedRecording_ = false;
  const int numFrames = getNumFrames();
  currentFrame_ = -1;

  while (currentFrame_ < numFrames - 1) {

    if (!pausedRecording_)
      currentFrame_++;

    if (breakpointFrame_ != 0 && currentFrame_ == breakpointFrame_)
      pausedRecording_ = true;

    size_t numPoints;
    PointT* points;
    getFrame(currentFrame_, numPoints, &points);

#ifdef USE_FRAMETIME
    const double frameTimeMs = frames_[currentFrame_].frameTime;
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(frameTimeMs));
#else
    const int frameTimeMs = _windowRecording->GetSliderIntValue(_sliderRecordingFrameTime);
    std::this_thread::sleep_for(std::chrono::milliseconds(frameTimeMs));
#endif

    _windowRecording->UpdateText(_textCurrentFrame, "Frame %d", (int)currentFrame_);

    if (numPoints != 0) {
      FrameDataPtr frameData(new FrameData(currentFrame_));
      PointCloudT* pc = new PointCloudT;
      pc->insert(std::begin(*pc), points, points + numPoints);

      PointCloudPtr pcp = pc->makeShared();
      frameData->cloud = pcp;
      frameData->cloudMinusSurfaces = pcp;
      notifyObservers(frameData);
    }

    const int curFrame = currentFrame_;

    while (pausedRecording_ && playingRecording_) {

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (currentFrame_ != curFrame) {
        break;
      }
    }

    if (!playingRecording_)
      return;
  }
}

}

#endif //LEPP3_RECORDER_HPP
