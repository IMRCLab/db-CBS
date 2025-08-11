#include "init_guess_mujoco.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <algorithm>
#include "dynobench/general_utils.hpp"
// Pad a matrix to match the maximum number of rows by repeating the last row
Eigen::MatrixXd pad_mat(const Eigen::MatrixXd &matrix, int maxRows) {
    Eigen::MatrixXd padded = Eigen::MatrixXd::Zero(maxRows, matrix.cols());
    int currentRows = matrix.rows();

    // Copy existing rows
    padded.topRows(currentRows) = matrix;

    // Pad remaining rows with the last row
    if (currentRows > 0) {
        for (int i = currentRows; i < maxRows; ++i) {
            padded.row(i) = matrix.row(currentRows - 1);
        }
    }

    return padded;
}

// Clip actions to a threshold
Eigen::MatrixXd clip_act(const Eigen::MatrixXd &actions, double threshold) {
    Eigen::MatrixXd clipped = actions;
    for (int i = 0; i < clipped.rows(); ++i) {
        for (int j = 0; j < clipped.cols(); ++j) {
            clipped(i, j) = std::clamp(clipped(i, j), 0.0, threshold);
        }
    }
    return clipped;
}

void generate_init_guess_mujoco(std::string &envPath, 
                                 std::string &payloadPath, 
                                 std::string &dbcbsPath, 
                                 std::string &resultPath, 
                                 size_t numRobots,
                                 std::string &joint_robot_env_path) 
{
    YAML::Node env = YAML::LoadFile(envPath);
    YAML::Node dbcbs = YAML::LoadFile(dbcbsPath);
    std::string robotname = env["joint_robot"][0]["type"].as<std::string>();
    std::cout << "generating init guess for: " << envPath << ", and robot name is: " << robotname<< std::endl;
    int maxRowsStates = 0, maxRowsActions = 0;
    std::vector<Eigen::MatrixXd> robotStates(numRobots), robotActions(numRobots);

    // === Load robot states/actions ===
    for (size_t i = 0; i < numRobots; ++i) {
        auto states = dbcbs["result"][i]["states"];
        auto actions = dbcbs["result"][i]["actions"];

        Eigen::MatrixXd stateMatrix(states.size(), states[0].size());
        for (size_t r = 0; r < states.size(); ++r)
            for (size_t c = 0; c < states[r].size(); ++c)
                stateMatrix(r, c) = states[r][c].as<double>();

        Eigen::MatrixXd actionMatrix(actions.size(), actions[0].size());
        for (size_t j = 0; j < actions.size(); ++j) {
            for (size_t k = 0; k < actions[j].size(); ++k) {
                actionMatrix(j, k) = actions[j][k].as<double>();
            }
        }
        std::cout << "robot " << i << " state size: " 
          << stateMatrix.rows() << "x" << stateMatrix.cols() << std::endl;

        robotStates[i] = stateMatrix;
        robotActions[i] = clip_act(actionMatrix, 1.4);

        maxRowsStates = std::max(maxRowsStates, static_cast<int>(stateMatrix.rows()));
        maxRowsActions = std::max(maxRowsActions, static_cast<int>(actionMatrix.rows()));
    }

    // Pad
    std::vector<Eigen::MatrixXd> paddedRobotActions(numRobots);
    for (size_t i = 0; i < numRobots; ++i) {
        robotStates[i] = pad_mat(robotStates[i], maxRowsStates);
        paddedRobotActions[i] = pad_mat(robotActions[i], maxRowsActions);
    }

    // Concatenate actions
    Eigen::MatrixXd concatenatedActions = Eigen::MatrixXd::Zero(maxRowsActions, numRobots * paddedRobotActions[0].cols());
    for (size_t i = 0; i < numRobots; ++i) {
        concatenatedActions.block(0, i * paddedRobotActions[i].cols(), maxRowsActions, paddedRobotActions[i].cols()) = paddedRobotActions[i];
    }
    // === Build new state layout ===
    Eigen::MatrixXd finalStates;


    if (startsWith(robotname, "mujocoquadspayload")) {
        YAML::Node payloadYaml = YAML::LoadFile(payloadPath);
        auto payloadInit = payloadYaml["payload"].as<std::vector<std::vector<double>>>();
        // Create matrix from payloadInit
        Eigen::MatrixXd payloadPosMatrix(payloadInit.size(), payloadInit[0].size());
        for (size_t i = 0; i < payloadInit.size(); ++i) {
            payloadPosMatrix.row(i) = Eigen::VectorXd::Map(payloadInit[i].data(), payloadInit[i].size());
        }


        // Allocate final stacked states
        finalStates.resize(
            maxRowsStates,
            3 + 4 +                     // payload pos + quat
            numRobots * (3 + 4) +        // quad pos + quat
            3 + 3 +                      // payload vel + ang vel
            numRobots * 6                // quad linear+angular vel
        );

        for (int t = 0; t < maxRowsStates; ++t) {
            int idx = 0;
            // Initial payload position
            Eigen::Vector3d payload_pos = payloadPosMatrix.row(t).head<3>();

            // // Adjust payload position if too far from any quad
            // for (size_t r = 0; r < numRobots; ++r) {
            //     Eigen::Vector3d quad_pos = robotStates[r].row(t).head<3>();

            //     Eigen::Vector3d diff = payload_pos - quad_pos;
            //     double dist = diff.norm();
            //     if (dist > 0.5) {
            //         diff *= (0.45 / dist);  // shrink vector to 0.5
            //         payload_pos = quad_pos + diff; // move payload closer to quad
            //     }
            // }

            // Write payload position
            finalStates.row(t).segment(idx, 3) = payload_pos;
            idx += 3;

            // Payload quaternion (always identity)
            finalStates.row(t).segment(idx, 4) << 0, 0, 0, 1;
            idx += 4;

            // Quad positions + quaternions
            for (size_t r = 0; r < numRobots; ++r) {
                finalStates.row(t).segment(idx, 3) = robotStates[r].row(t).head<3>();
                idx += 3;
                // finalStates.row(t).segment(idx, 4) = robotStates[r].row(t).segment<4>(3);
                finalStates.row(t).segment(idx, 4)<< 0, 0, 0, 1;
                idx += 4;
            }

            // Payload velocities (always zero)
            finalStates.row(t).segment(idx, 3) = Eigen::Vector3d::Zero();
            idx += 3;
            finalStates.row(t).segment(idx, 3) = Eigen::Vector3d::Zero();
            idx += 3;

            // Quad velocities: linear vel (cols 7–9) and angular vel (cols 10–12)
            for (size_t r = 0; r < numRobots; ++r) {
                // finalStates.row(t).segment(idx, 3) = robotStates[r].row(t).segment<3>(7);  // lin vel
                finalStates.row(t).segment(idx, 3) = Eigen::Vector3d::Zero();
                idx += 3;
                // finalStates.row(t).segment(idx, 3) = robotStates[r].row(t).segment<3>(10); // ang vel
                finalStates.row(t).segment(idx, 3) = Eigen::Vector3d::Zero();
                idx += 3;
            }

        }
        std::cout << "finished writing the initial guess" << std::endl;
    } else if (startsWith(robotname, "mujocoquad")) {
        finalStates = robotStates[0];
            // Force quaternion to identity (0,0,0,1) for all timesteps
        // for (int t = 0; t < finalStates.rows(); ++t) {
        //     finalStates.row(t).segment<4>(3) << 0, 0, 0, 1;
        //     finalStates.row(t).segment<3>(10) << 0, 0, 0;
        // }
    }
    else {
        throw std::runtime_error("Unsupported robot type: " + robotname);
    }

    if (!env["joint_robot"] || env["joint_robot"].size() == 0)
        throw std::runtime_error("joint_robot key is missing or empty in the provided environment YAML.");

    YAML::Node robotsNode = env["joint_robot"];
    YAML::Node robotsNode_traj_checker = YAML::Clone(env["joint_robot"]);

    YAML::Node outputEnvYaml = YAML::Clone(env);
    // outputEnvYaml["environment"]["max"][2] = 1000.;
    // outputEnvYaml["environment"]["min"][2] = -1000.;
    outputEnvYaml.remove("joint_robot");
    outputEnvYaml["robots"] = robotsNode;

    YAML::Node outputEnvYaml_traj_checker = YAML::Clone(env);
    outputEnvYaml_traj_checker.remove("joint_robot");
    outputEnvYaml_traj_checker["robots"] = robotsNode_traj_checker;
    std::string typeField = outputEnvYaml_traj_checker["robots"][0]["type"].as<std::string>();
    outputEnvYaml_traj_checker["robots"][0]["type"] = typeField.substr(0, typeField.find_last_of('.')) + "_traj_checker";

    std::string envOutputPath = resultPath.substr(0, resultPath.find_last_of("/\\") + 1) + "env.yaml";
    joint_robot_env_path = envOutputPath;
    std::ofstream envOutFile(envOutputPath);
    envOutFile << outputEnvYaml;

    std::string envOutputPath_traj_checker = resultPath.substr(0, resultPath.find_last_of("/\\") + 1) + "env_traj_checker.yaml";
    std::ofstream envOutFile_traj_checker(envOutputPath_traj_checker);
    envOutFile_traj_checker << outputEnvYaml_traj_checker;

    // === Save result.yaml ===
    YAML::Node result;
    YAML::Node statesNode, actionsNode;

    // Save states
    for (size_t i = 0; i < finalStates.rows(); ++i) {
        YAML::Node stateRow;
        for (int j = 0; j < finalStates.cols(); ++j) {
            stateRow.push_back(finalStates(i, j));
        }
        statesNode.push_back(stateRow);
    }

    for (size_t i = 0; i < concatenatedActions.rows(); ++i) {
        YAML::Node actionRow;
        for (int j = 0; j < concatenatedActions.cols(); ++j) {
            actionRow.push_back(concatenatedActions(i, j));
        }
        actionsNode.push_back(actionRow);
    }

    result["result"]["states"] = statesNode;
    result["result"]["actions"] = actionsNode;
    result["result"]["num_action"] = static_cast<int>(concatenatedActions.rows());
    result["result"]["num_states"] = static_cast<int>(finalStates.rows());

    // Save file
    std::ofstream resultOutFile(resultPath);
    resultOutFile << result;
    resultOutFile.close();
    std::cout << "Init guess generation complete: " << resultPath << "\n";
}
