//
//  Policy.hpp
//  MCMAS_UTM_DOMAIN
//
//  Created by Scott S Forer on 3/31/18.
//  Copyright © 2018 MCMAS_UTM_DOMAIN. All rights reserved.
//

#ifndef Policy_hpp
#define Policy_hpp

#include <iostream>
#include <cstdlib>
#include <vector>
#include <cmath>
#include <time.h>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <cassert>
#include <ctime>
#include <sstream>


using namespace std;


class Policy
{
    friend class Parameters;
    friend class Team;
    friend class Agent;
    friend class CCEA;
    friend class Waypoint;
    friend class Simulator;
    
protected:
    
    
public:
    vector<Waypoint> waypoint;
    int team_ID;
    int agent_ID;
    int policy_ID;
    vector<int> conflict_counter;
    vector<int> behavior_states;
    double collison_fitness;
    double paccet_fitness;
    double agent_fitness;
    double team_fitness;
    double fitness;
    
    
    //state information
    int target_waypoint;
    double distance_to_target_waypoint;
    double current_speed;
    vector<double> current_state;           //vector of x, y, and z coordinates
    vector<double> projected_state;         //vector of x, y, and z coordinates
    vector<double> inc_proj_state;
    vector<vector<double>> state_history;
    bool at_final_waypoint;
    
    
    
private:
    
    
};


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// Current Issues
//
//////// Resloved Issues
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////

#endif /* Policy_hpp */
