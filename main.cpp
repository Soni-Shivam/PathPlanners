#include <iostream>
#include <vector>
#include <fstream>      
#include <sstream>      
#include <string>       
#include <iomanip>
#include <cstdlib>     

#include "PAPF_Planner.h" 

// --- Helper Function to Load Obstacles from CSV ---
std::vector<Obstacle> loadObstaclesFromCSV(const std::string& filename) {
    std::vector<Obstacle> obstacles;
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open obstacle file: " << filename << std::endl;
        return obstacles; // Return empty vector
    }

    std::string line;
    // Read and discard the header line
    if (!std::getline(file, line)) {
        std::cerr << "Error: CSV file is empty or header is missing." << std::endl;
        return obstacles;
    }

    // Read the rest of the file
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        
        Obstacle obs;
        
        // Parse x, y, and radius
        try {
            // Get X
            std::getline(ss, value, ',');
            obs.center.x = std::stod(value);
            
            // Get Y
            std::getline(ss, value, ',');
            obs.center.y = std::stod(value);

            // Get Radius
            std::getline(ss, value, ',');
            obs.radius = std::stod(value);

            obstacles.push_back(obs);
        } catch (const std::invalid_argument& e) {
            std::cerr << "Warning: Skipping invalid line in CSV: " << line << std::endl;
        }
    }

    std::cout << "Successfully loaded " << obstacles.size() << " obstacles from " << filename << std::endl;
    file.close();
    return obstacles;
}

// --- NEW: Helper Function to Load Start/Goal from config file ---
bool loadScenarioConfig(const std::string& filename, USV& usv, Vector2D& goal) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open scenario config file: " << filename << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string key, value;
        
        if (std::getline(ss, key, ':') && std::getline(ss, value)) {
            value.erase(0, value.find_first_not_of(" \t"));

            try {
                if (key == "start_x") usv.position.x = std::stod(value);
                else if (key == "start_y") usv.position.y = std::stod(value);
                else if (key == "goal_x") goal.x = std::stod(value);
                else if (key == "goal_y") goal.y = std::stod(value);
            } catch (const std::invalid_argument& e) {
                std::cerr << "Warning: Skipping invalid value for key '" << key << "' in " << filename << std::endl;
            }
        }
    }
    file.close();
    std::cout << "Successfully loaded scenario from " << filename << std::endl;
    return true;
}


int main() {
    USV usv;
    Vector2D goal;

    // --- Load scenario from file
    if (!loadScenarioConfig("scenario.txt", usv, goal)) {
        std::cerr << "Scenario loading failed. Exiting." << std::endl;
        return 1;
    }
    
    // Set other initial USV properties
    usv.yaw = 0.0; 
    usv.velocity = 0.0;
    
    // Load Obstacles from CSV file
    std::vector<Obstacle> obstacles = loadObstaclesFromCSV("obstacles.csv");
    if (obstacles.empty()) {
        std::cerr << "Obstacle loading failed. Exiting simulation." << std::endl;
        return 1;
    }

    PAPF_Planner::Params params;
    PAPF_Planner planner(params);

    std::ofstream outFile("path_data.csv");
    outFile << "step,x,y,yaw_deg,velocity\n";

    // sim
    const int max_steps = 500000;
    for (int i = 0; i < max_steps; ++i) {
        planner.computeStep(usv, goal, obstacles);

        outFile << i << ","
                << std::fixed << std::setprecision(4) << usv.position.x << ","
                << usv.position.y << ","
                << usv.yaw * 180.0 / M_PI << ","
                << usv.velocity << "\n";

        if (usv.position.distanceTo(goal) < 0.1) {
            std::cout << "Goal reached in " << i << " steps.\n";
            break;
        }

        if (i == max_steps - 1) {
            std::cout << "Max steps reached.\n";
        }
    }

    outFile.close();
    std::cout << "path data saved to path_data.csv\n";


     // Execute the Python visualization script ---
    std::cout << "running viss" << std::endl;
    system("python3 viz2.py"); 

    return 0;
}