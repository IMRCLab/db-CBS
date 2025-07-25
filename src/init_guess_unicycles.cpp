#include "init_guess_unicycles.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <algorithm>


// Pad a matrix to match the maximum number of rows by repeating the last row
Eigen::MatrixXd pad_unicycles_matrix(const Eigen::MatrixXd &matrix, int maxRows) {
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

// Save a YAML file
void save_yaml(const std::string &file_path, const YAML::Node &data) {
    std::ofstream outFile(file_path);
    outFile << data;
    outFile.close();
}

void generate_init_guess_unicycles(std::string &envPath,
                        std::string &dbcbsPath, 
                        std::string &resultPath, 
                         size_t numRobots,
                        std::string &joint_robot_env_path) {
    // Load environment YAML
    YAML::Node env = YAML::LoadFile(envPath);

    // Load db_cbs YAML
    YAML::Node dbcbs = YAML::LoadFile(dbcbsPath);

    // Parse robot states and actions
    std::vector<Eigen::MatrixXd> robotStates(numRobots), robotActions(numRobots);
    int maxRowsStates = 0, maxRowsActions = 0;

    for (size_t i = 0; i < numRobots; ++i) {
        auto states = dbcbs["result"][i]["states"];
        auto actions = dbcbs["result"][i]["actions"];

        // Convert YAML to Eigen matrices
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
        robotActions[i] = actionMatrix;

        maxRowsStates = std::max(maxRowsStates, static_cast<int>(stateMatrix.rows()));
        maxRowsActions = std::max(maxRowsActions, static_cast<int>(actionMatrix.rows()));
    }

    // Pad states and actions to match the maximum size
    std::vector<Eigen::MatrixXd> paddedRobotStates(numRobots), paddedRobotActions(numRobots);
    for (size_t i = 0; i < numRobots; ++i) {
        paddedRobotStates[i] = pad_unicycles_matrix(robotStates[i], maxRowsStates);
        paddedRobotActions[i] = pad_unicycles_matrix(robotActions[i], maxRowsActions);
    }

    // Concatenate actions
    Eigen::MatrixXd concatenatedActions = Eigen::MatrixXd::Zero(maxRowsActions, numRobots * paddedRobotActions[0].cols());
    for (size_t i = 0; i < numRobots; ++i) {
        concatenatedActions.block(0, i * paddedRobotActions[i].cols(), maxRowsActions, paddedRobotActions[i].cols()) = paddedRobotActions[i];
    }

    // Initialize the joint states matrix
    Eigen::MatrixXd unicyclesJointStates = Eigen::MatrixXd::Zero(maxRowsStates, 2 + numRobots + (numRobots - 1));

    // Populate the joint states matrix
    for (size_t i = 0; i < maxRowsStates; ++i) {
        // Add the first robot's state
        unicyclesJointStates(i, 0) = paddedRobotStates[0](i, 0); // px1
        unicyclesJointStates(i, 1) = paddedRobotStates[0](i, 1); // py1
        unicyclesJointStates(i, 2) = paddedRobotStates[0](i, 2); // alpha1

        // Add alpha for remaining robots
        for (size_t j = 1; j < numRobots; ++j) {
            unicyclesJointStates(i, 2 + j) = paddedRobotStates[j](i, 2); // alpha_j
        }

        // Compute theta for each rod
        for (size_t j = 0; j < numRobots - 1; ++j) {
            Eigen::Vector2d pi = paddedRobotStates[j].row(i).head(2);
            Eigen::Vector2d pi_next = paddedRobotStates[j + 1].row(i).head(2);
            Eigen::Vector2d u = pi_next - pi;
            double theta = std::atan2(u.y(), u.x());

            // Add theta to the joint states
            unicyclesJointStates(i, 2 + numRobots + j) = theta;
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


    // Save results to YAML
    YAML::Node result;
    result["result"]["states"] = YAML::Node();
    result["result"]["actions"] = YAML::Node();

    for (int i = 0; i < unicyclesJointStates.rows(); ++i) {
        YAML::Node stateRow;
        for (int j = 0; j < unicyclesJointStates.cols(); ++j) {
            stateRow.push_back(unicyclesJointStates(i, j));
        }
        result["result"]["states"].push_back(stateRow);
    }

    for (int i = 0; i < concatenatedActions.rows(); ++i) {
        YAML::Node actionRow;
        for (int j = 0; j < concatenatedActions.cols(); ++j) {
            actionRow.push_back(concatenatedActions(i, j));
        }
        result["result"]["actions"].push_back(actionRow);
    }

    result["result"]["num_action"] = concatenatedActions.rows();
    result["result"]["num_states"] = unicyclesJointStates.rows();

    save_yaml(resultPath, result);

    std::cout << "Result saved to: " << resultPath << std::endl;


}
