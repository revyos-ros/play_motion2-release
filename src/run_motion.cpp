// Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <sstream>

#include "play_motion2/client.hpp"
#include "rclcpp/utilities.hpp"

void print_help()
{
  const auto help_message =
    "Usage: ros2 run play_motion2 run_motion <motion_name> <skip_planning> <timeout>\n"
    "\n"
    "Arguments:\n"
    "  <motion_name>    - Name of the motion to run.\n"
    "  <skip_planning>  - Whether to skip planning for approaching to the first position or not."
    " (default: false)\n"
    "  <timeout>        - Timeout (in seconds) to wait for the motion. (default: 120)\n"
    "  -h, --help       - Print this help message\n"
    "\n"
    "Example:\n"
    "$ ros2 run play_motion2 run_motion home false 30\n";

  std::cout << help_message << std::endl;
}

int main(int argc, char ** argv)
{
  // Check arguments
  if (argc < 2 || argc > 4) {
    std::cerr << "Error: Number of arguments is not valid.\n" << std::endl;
    print_help();
    return EXIT_FAILURE;
  }

  if (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") {
    print_help();
    return EXIT_SUCCESS;
  }

  const auto motion_key = argv[1];
  auto skip_planning = false;
  auto timeout = std::chrono::seconds(120);

  if (argc > 2) {
    std::stringstream ss(argv[2]);
    if (!(ss >> std::boolalpha >> skip_planning)) {
      std::cerr << "Error: The second argument must be 'true' or 'false'.\n" << std::endl;
      print_help();
      return EXIT_FAILURE;
    }
    if (argc > 3) {
      char * end;
      const auto seconds = strtol(argv[3], &end, 10);

      if (*end != '\0') {
        std::cerr << "Error: Third argument has non-numeric characters: "
                  << end << "\n" << std::endl;
        print_help();
        return EXIT_FAILURE;
      }
      timeout = std::chrono::seconds(seconds);
    }
  }

  // Create client and run motion
  rclcpp::init(argc, argv);

  auto client = std::make_shared<play_motion2::PlayMotion2Client>();

  client->run_motion(motion_key, skip_planning, timeout);

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
