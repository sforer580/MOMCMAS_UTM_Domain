//
//  CCEA.hpp
//  MOMCMAS_UTM_DOMAIN
//
//  Created by Scott S Forer on 3/31/18.
//  Copyright © 2018 MCMAS_UTM_DOMAIN. All rights reserved.
//

#ifndef CCEA_hpp
#define CCEA_hpp

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


class CCEA
{
    friend class Parameters;
    friend class Team;
    friend class Agent;
    friend class Waypoint;
    friend class Simulator;
    friend class Policy;
    
protected:
    
    
public:
    Parameters* pP;
    vector<Team> team;
    
    
    //CCEA functions
    void Init_Teams();
    void Init_Agents(int t);
    void Set_Behaviors(int t, int a, int p);
    void Init_Policies(int t, int a);
    void Init_Waypoints(int t, int a, int p);
    void Set_Starting_Waypoint();
    void Set_Intermediate_Waypoints();
    void Set_Final_Waypoint();
    void Set_Waypoints();
    void Initialize();
    void Shuffle_Policies();
    void Get_Fitness(vector<Policy> sim_policies);
    void Run_Leniency_Check(int p, int t, int a, int sp, vector<Policy> sim_policies);
    void Transfer_Simulation_Data(int p, vector<Policy> sim_policies);
    void Simulate(int p, vector<Policy> sim_policies);
    void Build_Simulated_Policies();
    void Evaluate();
    int Binary_Selection(int t, int a);
    void Down_Select();
    void Check_New_Waypoint(vector<double> coord_to_check);
    void Mutate(Policy M);
    void Repopulate();
    void Run_CCEA();
    void Run_Program();
    
    
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
//Creates the number of teams
void CCEA::Init_Teams(){
    for (int t=0; t<pP->num_teams; t++){
        Team T;
        team.push_back(T);
        team.at(t).team_ID = t;
        //initialize the agents for each team
        Init_Agents(t);
    }
}


/////////////////////////////////////////////////////////////////
//Creates the number of agents
void CCEA::Init_Agents(int t){
    for (int a=0; a<pP->num_agents_for_teams.at(t); a++){
        Agent A;
        team.at(t).agent.push_back(A);
        team.at(t).agent.at(a).team_ID = t;
        team.at(t).agent.at(a).agent_ID = a;
        //initialize the policies for each agent
        Init_Policies(t, a);
        
    }
}


/////////////////////////////////////////////////////////////////
//Sets the behaviors for each policy
void CCEA::Set_Behaviors(int t, int a, int p){
    if(t==0){
        team.at(t).agent.at(a).policy.at(p).behavior_states.push_back(0);
        team.at(t).agent.at(a).policy.at(p).behavior_states.push_back(pP->T0_T1_behavior);
        team.at(t).agent.at(a).policy.at(p).behavior_states.push_back(pP->T0_T2_behavior);
    }
    if(t==1){
        team.at(t).agent.at(a).policy.at(p).behavior_states.push_back(pP->T1_T0_behavior);
        team.at(t).agent.at(a).policy.at(p).behavior_states.push_back(0);
        team.at(t).agent.at(a).policy.at(p).behavior_states.push_back(pP->T1_T2_behavior);
    }
    if(t==2){
        team.at(t).agent.at(a).policy.at(p).behavior_states.push_back(pP->T2_T0_behavior);
        team.at(t).agent.at(a).policy.at(p).behavior_states.push_back(pP->T2_T1_behavior);
        team.at(t).agent.at(a).policy.at(p).behavior_states.push_back(0);
    }
    //cout << "cp" << endl;
}


/////////////////////////////////////////////////////////////////
//Creates the number of policies
void CCEA::Init_Policies(int t, int a){
    for (int p=0; p<pP->num_policies; p++){
        Policy Po;
        team.at(t).agent.at(a).policy.push_back(Po);
        team.at(t).agent.at(a).policy.at(p).team_ID = t;
        team.at(t).agent.at(a).policy.at(p).agent_ID = a;
        team.at(t).agent.at(a).policy.at(p).policy_ID = p;
        team.at(t).agent.at(a).policy.at(p).conflict_counter.resize(pP->num_teams);
        team.at(t).agent.at(a).policy.at(p).collison_fitness = 5485933;
        team.at(t).agent.at(a).policy.at(p).team_fitness = 5485933;
        team.at(t).agent.at(a).policy.at(p).agent_fitness = 5485933;
        team.at(t).agent.at(a).policy.at(p).paccet_fitness = 5485933;
        team.at(t).agent.at(a).policy.at(p).fitness = 5485933;
        team.at(t).agent.at(a).policy.at(p).current_state.resize(3);
        team.at(t).agent.at(a).policy.at(p).projected_state.resize(3);
        team.at(t).agent.at(a).policy.at(p).inc_proj_state_x_coord.resize((pP->ca_inc));
        team.at(t).agent.at(a).policy.at(p).inc_proj_state_y_coord.resize((pP->ca_inc));
        team.at(t).agent.at(a).policy.at(p).inc_proj_state_z_coord.resize((pP->ca_inc));
        Set_Behaviors(t, a, p);
        //initialize the waypoints for each policy
        Init_Waypoints(t, a, p);
    }
}


/////////////////////////////////////////////////////////////////
//Creates the number of waypoints
void CCEA::Init_Waypoints(int t, int a, int p){
    for (int w=0; w<pP->num_waypoints; w++){
        Waypoint W;
        team.at(t).agent.at(a).policy.at(p).waypoint.push_back(W);
        team.at(t).agent.at(a).policy.at(p).waypoint.at(w).waypoint_ID = w;
        //x, y, and z
        team.at(t).agent.at(a).policy.at(p).waypoint.at(w).coordinate.resize(3);
    }
}


/////////////////////////////////////////////////////////////////
//Sets the starting waypoint such that each policy for an agent has the same starting waypoints
void CCEA::Set_Starting_Waypoint(){
    for (int t=0; t<pP->num_teams; t++){
        for (int a=0; a<pP->num_agents_for_teams.at(t); a++){
            double x_waypoint = abs(((double)rand()/RAND_MAX)*pP->max_x_map);
            double y_waypoint = abs(((double)rand()/RAND_MAX)*pP->max_y_map);
            //int z_waypoint = abs(((double)rand()/RAND_MAX)*max_z_dim);
            //int x_waypoint = 2;
            //int y_waypoint = 4;
            double z_waypoint = 0;
            assert(pP->min_x_map<=x_waypoint<=pP->max_x_map);
            assert(pP->min_y_map<=y_waypoint<=pP->max_y_map);
            assert(pP->min_z_map<=z_waypoint<=pP->max_z_map);
            team.at(t).agent.at(a).policy.at(0).waypoint.at(0).coordinate.at(0) = x_waypoint;
            team.at(t).agent.at(a).policy.at(0).waypoint.at(0).coordinate.at(1) = y_waypoint;
            team.at(t).agent.at(a).policy.at(0).waypoint.at(0).coordinate.at(2) = z_waypoint;
        }
    }
    for (int t=0; t<pP->num_teams; t++){
        for (int a=0; a<pP->num_agents_for_teams.at(t); a++){
            for (int p=0; p<pP->num_policies; p++){
                double x_waypoint = team.at(t).agent.at(a).policy.at(0).waypoint.at(0).coordinate.at(0);
                double y_waypoint = team.at(t).agent.at(a).policy.at(0).waypoint.at(0).coordinate.at(1);
                double z_waypoint = team.at(t).agent.at(a).policy.at(0).waypoint.at(0).coordinate.at(2);
                team.at(t).agent.at(a).policy.at(p).waypoint.at(0).coordinate.at(0) = x_waypoint;
                team.at(t).agent.at(a).policy.at(p).waypoint.at(0).coordinate.at(1) = y_waypoint;
                team.at(t).agent.at(a).policy.at(p).waypoint.at(0).coordinate.at(2) = z_waypoint;
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//Sets the final waypoint such that each policy for an agent has the same final waypoints
void CCEA::Set_Intermediate_Waypoints(){
    for (int t=0; t<pP->num_teams; t++){
        for (int a=0; a<pP->num_agents_for_teams.at(t); a++){
            for (int p=0; p<pP->num_policies; p++){
                for (int w=1; w<pP->num_waypoints-1; w++)
                {
                    double x_waypoint = abs(((double)rand()/RAND_MAX)*pP->max_x_map);
                    double y_waypoint = abs(((double)rand()/RAND_MAX)*pP->max_y_map);
                    double z_waypoint = abs(((double)rand()/RAND_MAX)*pP->max_z_map);
                    assert(pP->min_x_map<=x_waypoint<=pP->max_x_map);
                    assert(pP->min_y_map<=y_waypoint<=pP->max_y_map);
                    assert(pP->min_z_map<=z_waypoint<=pP->max_z_map);
                    team.at(t).agent.at(a).policy.at(p).waypoint.at(w).coordinate.at(0) = x_waypoint;
                    team.at(t).agent.at(a).policy.at(p).waypoint.at(w).coordinate.at(1) = y_waypoint;
                    team.at(t).agent.at(a).policy.at(p).waypoint.at(w).coordinate.at(2) = z_waypoint;
                }
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//Sets the final waypoint such that each policy for an agent has the same final waypoints
void CCEA::Set_Final_Waypoint(){
    for (int t=0; t<pP->num_teams; t++){
        for (int a=0; a<pP->num_agents_for_teams.at(t); a++){
            double x_waypoint = abs(((double)rand()/RAND_MAX)*pP->max_x_map);
            double y_waypoint = abs(((double)rand()/RAND_MAX)*pP->max_y_map);
            //int z_waypoint = abs(((double)rand()/RAND_MAX)*max_z_dim);
            //int x_waypoint = 2;
            //int y_waypoint = 4;
            double z_waypoint = 0;
            assert(pP->min_x_map<=x_waypoint<=pP->max_x_map);
            assert(pP->min_y_map<=y_waypoint<=pP->max_y_map);
            assert(pP->min_z_map<=z_waypoint<=pP->max_z_map);
            team.at(t).agent.at(a).policy.at(0).waypoint.at(pP->num_waypoints-1).coordinate.at(0) = x_waypoint;
            team.at(t).agent.at(a).policy.at(0).waypoint.at(pP->num_waypoints-1).coordinate.at(1) = y_waypoint;
            team.at(t).agent.at(a).policy.at(0).waypoint.at(pP->num_waypoints-1).coordinate.at(2) = z_waypoint;
        }
    }
    for (int t=0; t<pP->num_teams; t++){
        for (int a=0; a<pP->num_agents_for_teams.at(t); a++){
            for (int p=0; p<pP->num_policies; p++){
                double x_waypoint = team.at(t).agent.at(a).policy.at(0).waypoint.at(pP->num_waypoints-1).coordinate.at(0);
                double y_waypoint = team.at(t).agent.at(a).policy.at(0).waypoint.at(pP->num_waypoints-1).coordinate.at(1);
                double z_waypoint = team.at(t).agent.at(a).policy.at(0).waypoint.at(pP->num_waypoints-1).coordinate.at(2);
                team.at(t).agent.at(a).policy.at(p).waypoint.at(pP->num_waypoints-1).coordinate.at(0) = x_waypoint;
                team.at(t).agent.at(a).policy.at(p).waypoint.at(pP->num_waypoints-1).coordinate.at(1) = y_waypoint;
                team.at(t).agent.at(a).policy.at(p).waypoint.at(pP->num_waypoints-1).coordinate.at(2) = z_waypoint;
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//Sets all the waypoints
void CCEA::Set_Waypoints(){
    Set_Starting_Waypoint();
    Set_Intermediate_Waypoints();
    Set_Final_Waypoint();
}


/////////////////////////////////////////////////////////////////
//Runs the initilaization process
void CCEA::Initialize(){
    Init_Teams();
    Set_Waypoints();
}


/////////////////////////////////////////////////////////////////
//Randomly Suffles the policies for each agent
void CCEA::Shuffle_Policies(){
    for (int t=0; t< pP->num_teams; t++){
        for (int a=0; a<pP->num_agents_for_teams.at(t); a++){
            random_shuffle (team.at(t).agent.at(a).policy.begin(), team.at(t).agent.at(a).policy.end());
        }
    }
}


/////////////////////////////////////////////////////////////////
//gets the fitness for each policy that was simulated
void CCEA::Get_Fitness(vector<Policy> sim_policies){
    //can add more objectives and PaCcET here
    //operating under single objective for now
    for (int sp=0; sp<pP->num_sim_policies; sp++){
        sim_policies.at(sp).fitness = sim_policies.at(sp).collison_fitness;
    }
}


/////////////////////////////////////////////////////////////////
//Runs Leniency
void CCEA::Run_Leniency_Check(int p, int t, int a, int sp, vector<Policy> sim_policies){
    if (sim_policies.at(sp).fitness < team.at(t).agent.at(a).policy.at(p).fitness){
        team.at(t).agent.at(a).policy.at(p).fitness = sim_policies.at(sp).fitness;
    }
}


/////////////////////////////////////////////////////////////////
//Transfers the simulation data to the dedicated storage space for each agent
void CCEA::Transfer_Simulation_Data(int p, vector<Policy> sim_policies){
    for (int sp=0; sp<pP->num_sim_policies; sp++){
        int t = sim_policies.at(sp).team_ID;
        int a  = sim_policies.at(sp).agent_ID;
        //run leniency here
        if (pP->leniency==true){
            Run_Leniency_Check(p, t, a, sp, sim_policies);
        }
        else{
            team.at(t).agent.at(sp).policy.at(p).fitness = sim_policies.at(sp).fitness;
        }
        team.at(t).agent.at(a).policy.at(p) = sim_policies.at(sp);
    }
    //cout << "cp" << endl;
}


/////////////////////////////////////////////////////////////////
//Creates the simulated policies
void CCEA::Simulate(int p,vector<Policy> sim_policies){
    Simulator S;
    S.pP = this->pP;
    vector<Policy>* psim_policy = &sim_policies;
    S.Run_Simulation(psim_policy);
    Get_Fitness(sim_policies);
    Transfer_Simulation_Data(p, sim_policies);
}


/////////////////////////////////////////////////////////////////
//Creates the simulated policies
void CCEA::Build_Simulated_Policies(){
    //this controls the lenincey loop
    for (int l=0; l<pP->len; l++){
        //this controls the simulation loop such that every policy is simulated
        for (int p=0; p<pP->num_policies; p++){
            vector<Policy> sim_policies;
            for (int t=0; t<pP->num_teams; t++){
                //sets the agent and policy information in the sim_agents vector
                for (int a=0; a<pP->num_agents_for_teams.at(t); a++){
                    //cout << pP->num_agents_for_teams.at(t) << endl;
                    sim_policies.push_back(team.at(t).agent.at(a).policy.at(p));
                }
            }
            assert (sim_policies.size()==pP->num_sim_policies);
            //At this point the simulated agents along with the policy information should be stored
            Simulate(p, sim_policies);
        }
    }
}


/////////////////////////////////////////////////////////////////
//Runs the evaluation process
void CCEA::Evaluate(){
    Shuffle_Policies();
    Build_Simulated_Policies();
}


/////////////////////////////////////////////////////////////////
//Randomly picks two policies for each agent and compares their fitness
int CCEA::Binary_Selection(int t, int a){
    int loser;
    int index_1 = rand() % team.at(t).agent.at(a).policy.size();
    int index_2 = rand() % team.at(t).agent.at(a).policy.size();
    while (index_1 == index_2){
        index_2 = rand() % team.at(t).agent.at(a).policy.size();
    }
    int f1 = team.at(t).agent.at(a).policy.at(index_1).fitness;
    int f2 = team.at(t).agent.at(a).policy.at(index_2).fitness;
    if (f1 < f2){
        loser = index_2;
    }
    else{
        loser = index_1;
    }
    return loser;
}


/////////////////////////////////////////////////////////////////
//Runs the downselect process
void CCEA::Down_Select(){
    for (int t=0; t<pP->num_teams; t++){
        for (int a=0; a<pP->num_agents_for_teams.at(t); a++){
            //will only run for half the num_policies
            for (int p=0; p<pP->to_kill; p++){
                int kill = 0;
                kill = Binary_Selection(t, a);
                //will erase the losing policy
                team.at(t).agent.at(a).policy.erase(team.at(t).agent.at(a).policy.begin() + kill);
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//Checks that the mutated waypoint is within the map parameters
void CCEA::Check_New_Waypoint(vector<double> coord_to_check){
    assert (coord_to_check.size()==3);
    if (coord_to_check.at(0) < pP->min_x_map){
        coord_to_check.at(0) = pP->min_x_map;
    }
    if (coord_to_check.at(0) > pP->max_x_map){
        coord_to_check.at(0) = pP->max_x_map;
    }
    if (coord_to_check.at(1) < pP->min_y_map){
        coord_to_check.at(0) = pP->min_y_map;
    }
    if (coord_to_check.at(1) > pP->max_y_map){
        coord_to_check.at(0) = pP->max_y_map;
    }
    if (coord_to_check.at(2) < pP->min_z_map){
        coord_to_check.at(0) = pP->min_z_map;
    }
    if (coord_to_check.at(2) > pP->max_z_map){
        coord_to_check.at(0) = pP->max_z_map;
    }
    assert (pP->min_x_map <= coord_to_check.at(0) <= pP->max_x_map);
    assert (pP->min_y_map <= coord_to_check.at(1) <= pP->max_y_map);
    assert (pP->min_z_map <= coord_to_check.at(2) <= pP->max_z_map);
}


/////////////////////////////////////////////////////////////////
//Runs the mutation process
void CCEA::Mutate(Policy M){
    //will only mutate the intermediate waypoints (not the staring or ending waypoints)
    for (int w=1; w<pP->num_waypoints-1; w++)
    {
        double rand_num = 0;
        rand_num = ((double) rand() / (RAND_MAX));       //double between 0 and 1
        //will only mutate the number of intermediate waypoints with a likelyhood = mutate_percentage
        if (rand_num<=pP->mutate_percentage/100)
        {
            for (int cp=0; cp<3; cp++)
            {
                double rand_sign = 0;
                rand_sign = ((double) rand() / (RAND_MAX));       //double between 0 and 1
                //if true will switch rand_sign to -1 else rand_sign = 1
                if (rand_sign<=0.5)
                {
                    rand_sign = -1;
                }
                else
                {
                    rand_sign = 1;
                }
                double original = 0;
                original = M.waypoint.at(w).coordinate.at(cp);
                original = original + (rand_sign)*(pP->mutation_range);
                M.waypoint.at(w).coordinate.at(cp) = original;
            }
            vector<double> coord_to_check = M.waypoint.at(w).coordinate;
            Check_New_Waypoint(coord_to_check);
        }
    }
}


/////////////////////////////////////////////////////////////////
//Runs the repopulate process
void CCEA::Repopulate(){
    for (int t=0; t<pP->num_teams; t++){
        for (int a=0; a<pP->num_agents_for_teams.at(t); a++){
            for (int p=0; p<pP->to_replicate; p++){
                Policy M;
                int spot = 0;
                spot = rand() % team.at(t).agent.at(a).policy.size();
                M = team.at(t).agent.at(a).policy.at(spot);
                Mutate(M);
                team.at(t).agent.at(a).policy.push_back(M);
            }
        }
    }
}


/////////////////////////////////////////////////////////////////
//Runs the entire CCEA
void CCEA::Run_CCEA(){
    Evaluate();
    Down_Select();
    Repopulate();
}


/////////////////////////////////////////////////////////////////
//Runs the entire program
void CCEA::Run_Program(){
    Initialize();
    for (int gen=0; gen<pP->gen_max; gen++){
        if (gen %10==0)
        {
            cout << "GEN" << "\t" << gen << endl;
        }
        Run_CCEA();
        //cout << "END GEN LOOP" << endl;
    }
    cout << "END STAT RUN" << endl;
}


#endif /* CCEA_hpp */
