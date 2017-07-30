/**
 * Copyright 2017 Massachusetts Institute of Technology
 *
 * @file utils.cc
 * @author W. Nicholas Greene
 * @date 2017-07-26 17:24:19 (Wed)
 */

#include "dataset_utils/utils.h"

#include <fstream>
#include <algorithm>
#include <sstream>

namespace dataset_utils {

std::vector<std::string> readLines(const std::string& file) {
  std::ifstream stream(file.c_str());
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(stream, line)) {
    // Remove carriage returns.
    line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
    lines.push_back(line);
  }

  return lines;
}

std::vector<std::string> split(const std::string &str, char delim) {
  std::vector<std::string> tokens;
  std::stringstream ss;
  ss.str(str);
  std::string token;
  while (std::getline(ss, token, delim)) {
    tokens.push_back(token);
  }

  return tokens;
}

}  // namespace dataset_utils
