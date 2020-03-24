#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>
#include <iostream>
#include <math.h>
#include <iterator>
//#include "helpers.h"
#define MAX_SPEED 49.5

class Vehicle {
public:
    // Constructors
    Vehicle();
    Vehicle(int lane,double x,double y, double vx, double vy,double s,double d, std::string state = "CS");

    // Destructor
    virtual ~Vehicle();

    Vehicle choose_next_state(std::map<int, std::vector<std::vector<double>> > predictions);

    // Get possible successor states
    std::vector<std::string> successor_states();

    std::vector<std::vector<double>> generate_predictions(int prev_path_size, int horizon = 2);

    double calculate_cost(std::string state, std::map<int, std::vector<std::vector<double>> > predictions, Vehicle* final_trajectory);

    std::map<std::string, int> lane_direction = { {"PLCL", -1}, {"LCL", -1},  {"LCR", 1}, {"PLCR", 1} };

    int lane, lanes_available;
    double x, y, vx, vy, s, d,ref_vel,speed,yaw,target_speed;

    std::string state;
};
#endif //VEHICLE_H