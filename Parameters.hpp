//
//  Parameters.hpp
//  MOMCMAS_UTM_DOMAIN
//
//  Created by Scott S Forer on 3/31/18.
//  Copyright © 2018 MCMAS_UTM_DOMAIN. All rights reserved.
//

#ifndef Parameters_hpp
#define Parameters_hpp

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


class Parameters
{
    friend class CCEA;
    friend class Team;
    friend class Agent;
    friend class Waypoint;
    friend class Simulator;
    friend class Policy;
    
protected:
    
    
public:
    //CCEA Parameters
    int num_teams = 3;
    int num_agents_for_team_0 = 2;
    int num_agents_for_team_1 = 2;
    int num_agents_for_team_2 = 2;
    vector<int> num_agents_for_teams;
    int num_policies = 5;
    int num_sim_policies;
    int num_waypoints = 3;
    bool leniency = true;
    int len = 1;
    int gen_max = 1;
    int to_kill = num_policies/2;
    int to_replicate = num_policies/2;
    double mutate_percentage = 50;
    double mutation_range = 1;
    
    //Behavior Modes
    int T0_T1_behavior = 1;                 //0=cooperative, 1=uncooperative, 2=malicious
    int T0_T2_behavior = 2;                 //0=cooperative, 1=uncooperative, 2=malicious
    int T1_T0_behavior = 2;                 //0=cooperative, 1=uncooperative, 2=malicious
    int T1_T2_behavior = 1;                 //0=cooperative, 1=uncooperative, 2=malicious
    int T2_T0_behavior = 1;                 //0=cooperative, 1=uncooperative, 2=malicious
    int T2_T1_behavior = 1;                 //0=cooperative, 1=uncooperative, 2=malicious
    vector<vector<int>> behavior_modes;
    
    
    //Simulator Parameters
    int min_x_map = 0;
    int max_x_map = 35;
    int min_y_map = 0;
    int max_y_map = 35;
    int min_z_map = 0;
    int max_z_map = 35;
    int time_max = 150;                                         //max time simulator will run
    double delta_t = 0.1;                                       //simulator time step
    int num_time_steps = time_max/delta_t;                      //number of time steps in a simulation
    double max_flight_velocity = 5.0;                           //max velocity at which any given agetn can travel
    double max_travel_dist = max_flight_velocity*delta_t;       //max distance a agetn can travel in a time step
    int ca_radius = 5;                                          //collision avoidance radius
    int ca_inc = 100;                                           //amount of increments between the current telem and projected telem
    double ca_flight_speed = max_flight_velocity/2;             //collision avoidance speed
    double ca_max_travel_dist = ca_flight_speed*delta_t;        //max distance any agent can travel when collision avoidance is acitvated
    
    
    
    
    //Parameter Functions
    void Set_Team_Sizes();
    void Calc_Num_Sim_Policies();
    void Init_Parameters();
    
    
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
//Sets the team sizes
void Parameters::Set_Team_Sizes(){
    num_agents_for_teams.push_back(num_agents_for_team_0);
    num_agents_for_teams.push_back(num_agents_for_team_1);
    num_agents_for_teams.push_back(num_agents_for_team_2);
}


/////////////////////////////////////////////////////////////////
//gets the number of simulated policies
void Parameters::Calc_Num_Sim_Policies(){
    num_sim_policies = 0;
    for (int t=0; t<num_teams; t++){
        num_sim_policies += num_agents_for_teams.at(t);
    }
}


/////////////////////////////////////////////////////////////////
//Sets all the parameters
void Parameters::Init_Parameters(){
    Set_Team_Sizes();
    Calc_Num_Sim_Policies();
}

#endif /* Parameters_hpp */
