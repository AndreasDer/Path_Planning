#include "vehicle.h"
// Initializes Vehicle
Vehicle::Vehicle() {
    lanes_available = 3;
    ref_vel = 0.0;
    target_speed = MAX_SPEED;
}

Vehicle::Vehicle(int lane, double x, double y, double vx, double vy, double s, double d, std::string state) {
    this->lane = lane;
    this->x = x;
    this->y = y;
    this->vx = vx;
    this->vy = vy;
    this->s = s;
    this->d = d;
    this->state = state;
    lanes_available = 3;
    ref_vel = 0.0;
}

Vehicle::~Vehicle() {}

Vehicle Vehicle::choose_next_state(std::map<int, std::vector<std::vector<double>>> predictions)
{
    bool too_close = false;
    std::string next_state = "KL";
    double final_cost = 999.0;
    Vehicle final_trajectory =*this;
    std::vector<std::string> states = successor_states();
    // Find next state with smallest cost
    for (int i = 0; i < states.size(); i++) {
        Vehicle trajectory=*this;
        double cost = calculate_cost(states[i], predictions, &trajectory);
        //std::cout << "State: " << states[i] << " cost: " <<cost<< ","<<trajectory.lane << ","<<trajectory.target_speed<< std::endl;

        if (cost < final_cost) {
            final_cost = cost;
            next_state = states[i];
            final_trajectory = trajectory;
        }
    }
    // If the speed in target lane is smaller than targeted speed, decrease targeted speed
    if (final_trajectory.target_speed < this->ref_vel) {
        this->ref_vel -= .224;
    }
    else if (ref_vel < 49.5) {
        this->ref_vel += .224;
    }
    this->state = next_state;
    this->lane = final_trajectory.lane;
    //std::cout << next_state << " ," << ref_vel << " , "<< final_trajectory.target_speed<<"," << lane <<","<<final_cost<<", "<< final_trajectory.lane<< std::endl;
    return final_trajectory;
}

std::vector<std::string> Vehicle::successor_states() {
    // Provides the possible next states given the current state for the FSM 
    std::vector<std::string> states;
    states.push_back("KL"); //Keep lane is always an option
    std::string state = this->state;
    if (state.compare("KL") == 0) {
        if(lane > 0)
            states.push_back("PLCL");
        if(lane < lanes_available-1)
            states.push_back("PLCR");
        //std::cout << "Lane: " << lane << " lanes av: " << lanes_available << std::endl;
    }
    else if (state.compare("PLCL") == 0) {
        if (lane != 0) {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    }
    else if (state.compare("PLCR") == 0) {
        if (lane != lanes_available - 1) {
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    }

    // If state is "LCL" or "LCR", then just return "KL"
    return states;
}

std::vector<std::vector<double>> Vehicle::generate_predictions(int prev_path_size, int horizon)
{
    // Generates predictions for non-ego vehicles to be used in trajectory 
    //   generation for the ego vehicle.
    // Returns the current position of the vehicle and an estimated position where the non-ego vehicle will be
    // after the timesteps it took since last prediction
    // Speed and lane are estimated to stay the same

    double check_speed = sqrt(vx * vx + vy * vy);
    double next_s = s;

    std::vector<std::vector<double>> predictions;
    std::vector<double> predictions_s;
    std::vector<double> predictions_d;
    std::vector<double> predictions_speed;
    predictions_s.push_back(next_s);
    predictions_d.push_back(d);
    predictions_speed.push_back(check_speed);
    next_s += ((double)prev_path_size * .02 * check_speed);
    predictions_s.push_back(next_s);
    predictions_d.push_back(d);
    predictions_speed.push_back(check_speed);
    predictions.push_back(predictions_s);
    predictions.push_back(predictions_d);
    predictions.push_back(predictions_speed);
    return predictions;
}

double Vehicle::calculate_cost(std::string state, std::map<int, std::vector<std::vector<double>>> predictions, Vehicle* trajectory)
{
    // Cost function for the transition from the actual state to the target state.
    std::map<int, std::vector<std::vector<double>> >::iterator it = predictions.begin();
    double cost = 0.0;
    bool target_lane_free = true;
    if (state == "KL") {
        while (it != predictions.end())
        {
            std::vector<std::vector<double>> prediction = it->second;
            double d = prediction[1][0];
            if (d<(2 + 4 * this->lane + 2) && d>(2 + 4 * this->lane - 2)) {

                if (((prediction[0][1] > this->s) && ((prediction[0][1] - this->s) < 30))|| fabs(prediction[0][0] - this->s)<10) {
                    trajectory->target_speed = prediction[2][0];
                    // If lane is not center lane increase cost to get vehicle to go in center lane to be able to switch to both other lanes
                    if (lane == 0 || lane == lanes_available - 1) {
                        cost += 1;
                    }
                }
            }
            it++;
        }
    }
    if (state == "LCL" || state == "LCR") {
        double new_lane = 0.0;
        while (it != predictions.end())
        {
            std::vector<std::vector<double>> prediction = it->second;
            double d = prediction[1][0];
            new_lane = this->lane + lane_direction[state];
            if (d<(2 + 4 * new_lane + 2) && d>(2 + 4 * new_lane - 2)) {

                if ((prediction[0][1] > this->s) && ((prediction[0][1] - this->s) < 25)) {
                    trajectory->target_speed = prediction[2][0];
                }
                //If  s position of non-ego vehicle in target lane is too close, increase cost to prevent collisions
                if (fabs((prediction[0][0] - this->s)) < 25) {
                    cost+=10;
                }
            }
            it++;
        }
        trajectory->lane = new_lane;
    }

    if (state == "PLCL" || state == "PLCR") {
        double lane_speed = 49.5;
        double target_lane_speed = 49.5;
        while (it != predictions.end())
        {
            std::vector<std::vector<double>> prediction = it->second;
            double d = prediction[1][0];
            int new_lane = this->lane + lane_direction[state];
            if (d<(2 + 4 * new_lane + 2) && d>(2 + 4 * new_lane - 2)) {
                // If target lane is occupied
                if ((prediction[0][1] > this->s) && ((prediction[0][1] - this->s) < 20)) {
                    if (prediction[2][0] < target_lane_speed) {
                        target_lane_speed = prediction[2][0];
                        target_lane_free = false;
                    }
                }
            }
            else if (d < (2 + 4 * this->lane + 2) && d>(2 + 4 * this->lane - 2)) {
                if ((prediction[0][1] > this->s) && ((prediction[0][1] - this->s) < 30)) {
                    if (prediction[2][0] < lane_speed) {
                        lane_speed = prediction[2][0];
                    }
                }
            }
            it++;
            // If target lane is free, increase cost for prepare lane change 
            if (target_lane_free) {
                cost += 1;
            }
        }
        // If target lane is occupied, stay in actual lane and adapt speed to actual lane
        if (lane_speed < target_lane_speed&&!target_lane_free) {
            trajectory->target_speed = lane_speed;
        }
        else {
            trajectory->target_speed = target_lane_speed;
        }
    }

    cost += MAX_SPEED - trajectory->target_speed;
    return cost;
}
