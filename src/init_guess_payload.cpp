#include "init_guess_payload.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <algorithm>

// Helper to normalize a vector
Eigen::VectorXd normalize(const Eigen::VectorXd &vec) {
    double norm = vec.norm();
    if (norm > 0) {
        return vec / norm;
    } else {
        throw std::runtime_error("Cannot divide by zero");
    }
}

// Clip actions to a threshold
Eigen::MatrixXd clip_actions(const Eigen::MatrixXd &actions, double threshold) {
    Eigen::MatrixXd clipped = actions;
    for (int i = 0; i < clipped.rows(); ++i) {
        for (int j = 0; j < clipped.cols(); ++j) {
            clipped(i, j) = std::clamp(clipped(i, j), 0.0, threshold);
        }
    }
    return clipped;
}

// Pad a matrix to match the maximum number of rows by repeating the last row
Eigen::MatrixXd pad_matrix(const Eigen::MatrixXd &matrix, int maxRows) {
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

// Main function for processing YAML and generating the result
void generate_init_guess_payload(std::string &envPath, 
                                 std::string &payloadPath, 
                                 std::string &dbcbsPath, 
                                 std::string &resultPath, 
                                 size_t numRobots,
                                 std::string &joint_robot_env_path) {
    // Load YAML files
    YAML::Node env = YAML::LoadFile(envPath);
    YAML::Node payload = YAML::LoadFile(payloadPath);
    YAML::Node dbcbs = YAML::LoadFile(dbcbsPath);

    // 1. Extract and pad states and actions to match the longest sequence
    std::vector<Eigen::MatrixXd> robotStates(numRobots), robotActions(numRobots);
    int maxRowsStates = 0;
    int maxRowsActions = 0;

    for (int i = 0; i < numRobots; ++i) {
        auto states = dbcbs["result"][i]["states"];
        auto actions = dbcbs["result"][i]["actions"];

        // Convert YAML to Eigen
        Eigen::MatrixXd stateMatrix(states.size(), states[0].size());
        Eigen::MatrixXd actionMatrix(actions.size(), actions[0].size());

        for (size_t j = 0; j < states.size(); ++j) {
            for (size_t k = 0; k < states[j].size(); ++k) {
                stateMatrix(j, k) = states[j][k].as<double>();
            }
        }
        for (size_t j = 0; j < actions.size(); ++j) {
            for (size_t k = 0; k < actions[j].size(); ++k) {
                actionMatrix(j, k) = actions[j][k].as<double>();
            }
        }

        robotStates[i] = stateMatrix;
        robotActions[i] = clip_actions(actionMatrix, 1.4);
        maxRowsStates = std::max(maxRowsStates, static_cast<int>(stateMatrix.rows()));
        maxRowsActions = std::max(maxRowsActions, static_cast<int>(actionMatrix.rows()));
    }

    // Pad states and actions
    std::vector<Eigen::MatrixXd> paddedRobotStates(numRobots), paddedRobotActions(numRobots);
    for (int i = 0; i < numRobots; ++i) {
        paddedRobotStates[i] = pad_matrix(robotStates[i], maxRowsStates);
        paddedRobotActions[i] = pad_matrix(robotActions[i], maxRowsActions);
    }

    // Concatenate actions
    Eigen::MatrixXd concatenatedActions = Eigen::MatrixXd::Zero(maxRowsActions, numRobots * paddedRobotActions[0].cols());
    for (size_t i = 0; i < numRobots; ++i) {
        concatenatedActions.block(0, i * paddedRobotActions[i].cols(), maxRowsActions, paddedRobotActions[i].cols()) = paddedRobotActions[i];
    }

    // 2. Build and initialize the payload states matrix
    Eigen::MatrixXd payloadStates = Eigen::MatrixXd::Zero(maxRowsStates, 6 + 6 * numRobots + 7 * numRobots);
    auto payloadInit = payload["payload"].as<std::vector<std::vector<double>>>();

    // 3. Compute the components and populate the payload states matrix
    for (size_t i = 0; i < maxRowsStates; ++i) {
        Eigen::VectorXd p0 = Eigen::VectorXd::Map(payloadInit[i].data(), payloadInit[i].size());
        payloadStates.row(i).head(3) = p0;

        for (int j = 0; j < numRobots; ++j) {
            Eigen::VectorXd pi = paddedRobotStates[j].row(i).head(3);
            Eigen::VectorXd qi = normalize(p0 - pi);
            payloadStates.row(i).segment(6 + 6 * j, 3) = qi;
            payloadStates.row(i).segment(6 + 6 * numRobots + 7 * j, 4) << 0, 0, 0, 1;
        }
    }

    // Copy the joint_robot key content into robots
    if (!env["joint_robot"] || env["joint_robot"].size() == 0) {
        throw std::runtime_error("joint_robot key is missing or empty in the provided environment YAML.");
    }

    YAML::Node robotsNode = env["joint_robot"];

    // Create the output environment YAML
    YAML::Node outputEnvYaml = env;
    outputEnvYaml.remove("joint_robot");
    outputEnvYaml["robots"] = robotsNode;

    // Save the new env.yaml file
    std::string envOutputPath = resultPath.substr(0, resultPath.find_last_of("/\\") + 1) + "env.yaml";
    joint_robot_env_path = envOutputPath;
    std::ofstream envOutFile(envOutputPath);
    envOutFile << outputEnvYaml;
    envOutFile.close();

    // Create result YAML
    YAML::Node result;
    YAML::Node statesNode, actionsNode;

    for (size_t i = 0; i < payloadStates.rows(); ++i) {
        YAML::Node stateRow;
        for (int j = 0; j < payloadStates.cols(); ++j) {
            stateRow.push_back(payloadStates(i, j));
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
    result["result"]["num_states"] = static_cast<int>(payloadStates.rows());

    // Save result YAML
    std::ofstream resultOutFile(resultPath);
    resultOutFile << result;
    resultOutFile.close();

    std::cout << "Payload processing completed and result saved to: " << resultPath << std::endl;
}
