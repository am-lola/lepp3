#ifndef LEPP3_VIDEO_RECORDER_H_
#define LEPP3_VIDEO_RECORDER_H_

#include <iostream>
#include <iterator>
#include <algorithm>
#include <vector>
#include <sstream>
#include <boost/filesystem.hpp>
#include <boost/iterator/filter_iterator.hpp>

namespace fs = boost::filesystem;

/**
 * Helper function to sort the files found in a directory.
 * All files follow the format: cloud_[NUMBER].pcd
 * Extracting the NUMBER gives the comparison criterion.
 */
bool comparator(std::string const& s1, std::string const& s2) {
  size_t first = s1.find("_");
  size_t last = s1.find(".pcd");
  std::string num1 = s1.substr(first+1,last-first-1);

  first = s2.find("_");
  last = s2.find(".pcd");
  std::string num2 = s2.substr(first+1,last-first-1);
  // convert string to int (C++98)
  // TODO: C++11 --> change to std::stoi()
  int n1, n2;
  std::istringstream(num1) >> n1;
  std::istringstream(num2) >> n2;

  return (n1 < n2);
}

class FileManager {
public:
  FileManager(const std::string& directory)
    : path_(directory),
      current_file_counter_(0) {}
  /**
   * This function receives the extension of the files we are looking for in
   * the current path_ and returns the name of all corresponding files in that
   * directory. The relative path is sent back to whoever requested.
   */
  std::vector<std::string> getFileNames(const std::string& extension);
private:
  fs::path path_;
  int current_file_counter_;
};

std::vector<std::string> FileManager::getFileNames(const std::string& ext) {
	int current_file_counter_ = 0;
  std::vector<std::string> file_names;
  // Default constructor for an iterator is the end iterator
	fs::directory_iterator end_iter;
	for (fs::directory_iterator iter(path_); iter != end_iter; ++iter)
		if (iter->path().extension() == ext) {
      std::string const fn = iter->path().filename().string();
      file_names.push_back(fn);
    	++current_file_counter_;
    }
  // sort files
  std::sort(file_names.begin(), file_names.end(), comparator);
  std::cout << "found " << current_file_counter_
            << " " << ext
            << " files in directory " << path_.relative_path() << std::endl;
  // append the parent directory path to the file name
  std::string parent = path_.parent_path().string();
  for(int i=0; i<current_file_counter_; ++i){
    std::stringstream ss;
    ss << parent << "/" << file_names[i];
    file_names[i] = ss.str();
  }
	return file_names;
}

#endif // LEPP3_VIDEO_RECORDER_H_
