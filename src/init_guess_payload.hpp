#pragma once

#include <string>

// Function declaration
void generate_init_guess_payload(std::string &envPath, 
                    std::string &payloadPath, 
                    std::string &dbcbsPath, 
                    std::string &resultPath, 
                    size_t numRobots,
                    std::string &joint_robot_env_path);
