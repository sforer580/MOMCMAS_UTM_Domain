//
//  Simulator.hpp
//  MOMCMAS_UTM_DOMAIN
//
//  Created by Scott S Forer on 3/31/18.
//  Copyright © 2018 MCMAS_UTM_DOMAIN. All rights reserved.
//

#ifndef Simulator_hpp
#define Simulator_hpp

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


class Simulator
{
    friend class Parameters;
    friend class Team;
    friend class Agent;
    friend class Waypoint;
    friend class CCEA;
    friend class Policy;
    
protected:
    
    
public:
    Parameters* pP;
    
    //Simulator Initialization Functions
    void Init_Conflict_Counter(int sp, vector<Policy>* pspolicy);           //Initializes the conflict counter for an agent
    void Init_Flight_Speed(int sp, vector<Policy>* pspolicy);               //Initializes the flight speed for an agent
    void Init_Target_Waypoint(int sp, vector<Policy>* pspolicy);            //Initializes the target waypoint for an agent
    void Init_Final_Waypoint_Statement(int sp, vector<Policy>* pspolicy);   //Initializes the final waypoint statement for an agent
    void Init_Current_State(int sp, vector<Policy>* pspolicy);              //Initializes the current state to waypoint 0 for an agent
    void Init_pspolicy(vector<Policy>* pspolicy);                           //Runs the Initialization process for each agent
    
    //Simulator Functions
    void Get_Dist_To_Target_Waypoint(vector<Policy>* pspolicy, int sp);     //Gets an agents distance to their target waypoint
    void Calc_Projected_State(vector<Policy>* pspolicy, int sp);            //Claculates the projected state for an agent
    void Get_Projected_State(vector<Policy>* pspolicy, int sp);             //Gets an sgents projected state based on its travel speed proximity to it
    void Get_Inc_Projected_State(vector<Policy>* pspolicy, int sp);         //Gets an agents incremented projected state
    double Get_Distance_To_Other_Agent(vector<Policy>* pspolicy, int sp, int spp, double distance); //Calculates the distance between to agents
    void Compare_Agents_Inc_Projected_State(vector<Policy>* pspolicy, int sp, int spp); //Compares two agents incremented projected states
    void Update_Conflict_Counter(vector<Policy>* sim_team, int sim_p, int sim_pp); //Updates the conflict counter for an agent
    void Check_for_Collisions(vector<Policy>* pspolicy);                    //Runs the collision check process
    void Collision_Detection(vector<Policy>* pspolicy);                     //Runs the collision detection process
    void Check_If_At_Target_Waypoint(vector<Policy>* pspolicy, int sp);     //Checks if an agent have reached their target waypoint
    void Set_Current_State(vector<Policy>* pspolicy);                       //Sets the current state for each agent to their projected state
    void Check_If_At_Final_Waypoint(vector<Policy>* pspolicy);              //Checks if each agent has reached their final waypoint
    void Run_Simulation(vector<Policy>* pspolicy);                          //Runs the entire simulation process
    
private:
    
    
};


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////// Current Issues
//Need a full test of the simulator that checks the following: 1) agents can be placed into the simulation, 2) agents can move to their target waypoints, 3) projected state calculations, 4) incremented projected state calculations, 5) distance to other agents 6) collision dectection, 7) conflict counter, and 8) agents can reach their final waypoint
//////// Resloved Issues
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////
//Initializes the conflict counter for an agent
void Simulator::Init_Conflict_Counter(int sp, vector<Policy>* pspolicy){
    for (int t=0; t<pP->num_teams; t++){
        pspolicy->at(sp).conflict_counter.at(t) = 0;
    }
}


/////////////////////////////////////////////////////////////////
//Initializes the flight speed for an agent
void Simulator::Init_Flight_Speed(int sp, vector<Policy>* pspolicy){
    for (int t=0; t<pP->num_teams; t++){
        pspolicy->at(sp).current_speed = pP->max_flight_velocity;
    }
}


/////////////////////////////////////////////////////////////////
//Initializes the target waypoint for an agent
void Simulator::Init_Target_Waypoint(int sp, vector<Policy>* pspolicy){
    for (int t=0; t<pP->num_teams; t++){
        pspolicy->at(sp).target_waypoint = 1;
    }
}


/////////////////////////////////////////////////////////////////
//Initializes the final waypoint statement for an agent
void Simulator::Init_Final_Waypoint_Statement(int sp, vector<Policy>* pspolicy){
    for (int t=0; t<pP->num_teams; t++){
        pspolicy->at(sp).at_final_waypoint = false;
    }
}


/////////////////////////////////////////////////////////////////
//Initializes the current state to waypoint 0 for an agent
void Simulator::Init_Current_State(int sp, vector<Policy>* pspolicy){
    for (int t=0; t<pP->num_teams; t++){
        pspolicy->at(sp).current_state = pspolicy->at(sp).waypoint.at(0).coordinate;
        pspolicy->at(sp).state_history.push_back(pspolicy->at(sp).current_state);
    }
}


/////////////////////////////////////////////////////////////////
//Runs the Initialization process for each agent
void Simulator::Init_pspolicy(vector<Policy>* pspolicy){
    for (int sp=0; sp<pspolicy->size(); sp++){
        Init_Conflict_Counter(sp, pspolicy);
        Init_Flight_Speed(sp, pspolicy);
        Init_Target_Waypoint(sp, pspolicy);
        Init_Current_State(sp, pspolicy);
    }
}


/////////////////////////////////////////////////////////////////
//Gets an agents distance to their target waypoint
void Simulator::Get_Dist_To_Target_Waypoint(vector<Policy>* pspolicy, int sp){
    pspolicy->at(sp).distance_to_target_waypoint = 0;
    double V_1;
    double V_2;
    double V_3;
    double V_mag_1;
    double V_mag_2;
    double V_mag_3;
    int P1 = pspolicy->at(sp).target_waypoint;
    V_1 = (pspolicy->at(sp).waypoint.at(P1).coordinate.at(0) - pspolicy->at(sp).current_state.at(0));
    V_2 = (pspolicy->at(sp).waypoint.at(P1).coordinate.at(1) - pspolicy->at(sp).current_state.at(1));
    V_3 = (pspolicy->at(sp).waypoint.at(P1).coordinate.at(2) - pspolicy->at(sp).current_state.at(2));
    V_mag_1 = V_1*V_1;
    V_mag_2 = V_2*V_2;
    V_mag_3 = V_3*V_3;
    pspolicy->at(sp).distance_to_target_waypoint = sqrt(V_mag_1 + V_mag_2 + V_mag_3);
}


/////////////////////////////////////////////////////////////////
//Claculates the projected state for an agent
void Simulator::Calc_Projected_State(vector<Policy>* pspolicy, int sp){
    double V_1;
    double V_2;
    double V_3;
    double D_1;
    double D_2;
    double D_3;
    double V_mag_1;
    double V_mag_2;
    double V_mag_3;
    double V_mag;
    double current_x;
    double current_y;
    double current_z;
    double travel_dist;
    int P1 = pspolicy->at(sp).target_waypoint;
    int P2 = pspolicy->at(sp).target_waypoint -1;
    
    //Calculates the travel distance
    travel_dist = pspolicy->at(sp).current_speed*pP->delta_t;
    
    //Creates Vector Between Waypoints
    V_1 = (pspolicy->at(sp).waypoint.at(P1).coordinate.at(0) - pspolicy->at(sp).waypoint.at(P2).coordinate.at(0));
    V_2 = (pspolicy->at(sp).waypoint.at(P1).coordinate.at(1) - pspolicy->at(sp).waypoint.at(P2).coordinate.at(1));
    V_3 = (pspolicy->at(sp).waypoint.at(P1).coordinate.at(2) - pspolicy->at(sp).waypoint.at(P2).coordinate.at(2));
    V_mag_1 = V_1*V_1;
    V_mag_2 = V_2*V_2;
    V_mag_3 = V_3*V_3;
    V_mag = sqrt(V_mag_1 + V_mag_2 + V_mag_3);
    
    //Creates Unit Vector Between Waypoints
    D_1 = (V_1/V_mag);
    D_2 = (V_2/V_mag);
    D_3 = (V_3/V_mag);
    
    //Calculates project state
    current_x = pspolicy->at(sp).current_state.at(0) + travel_dist*D_1;
    current_y = pspolicy->at(sp).current_state.at(1) + travel_dist*D_2;
    current_z = pspolicy->at(sp).current_state.at(2) + travel_dist*D_3;
    pspolicy->at(sp).projected_state.empty();
    pspolicy->at(sp).projected_state.at(0) = current_x;
    pspolicy->at(sp).projected_state.at(1) = current_y;
    pspolicy->at(sp).projected_state.at(2) = current_z;
}


/////////////////////////////////////////////////////////////////
//Gets an sgents projected state based on its travel speed and proximity to it
void Simulator::Get_Projected_State(vector<Policy>* pspolicy, int sp)
{
    if (pspolicy->at(sp).distance_to_target_waypoint > 0){
        if (pspolicy->at(sp).distance_to_target_waypoint < pspolicy->at(sp).current_speed*pP->delta_t){
            //cout << "cp3" << endl;
            //checks to see if the distance from the current state to the target waypoint is within the max travel distance
            vector<double> proj_state = pspolicy->at(sp).waypoint.at(pspolicy->at(sp).target_waypoint).coordinate;
            pspolicy->at(sp).projected_state = proj_state;
        }
    }
    else if (pspolicy->at(sp).distance_to_target_waypoint > 0){
        if (pspolicy->at(sp).distance_to_target_waypoint > pspolicy->at(sp).current_speed*pP->delta_t){
            //cout << "cp4" << endl;
            //calculates the new projected state
            Calc_Projected_State(pspolicy, sp);
        }
    }
    else{
        vector<double> proj_state = pspolicy->at(sp).waypoint.at(pspolicy->at(sp).target_waypoint).coordinate;
        pspolicy->at(sp).projected_state = proj_state;
    }
}


/////////////////////////////////////////////////////////////////
//Gets an agents incremented projected state
void Simulator::Get_Inc_Projected_State(vector<Policy>* pspolicy, int sp){
    pspolicy->at(sp).inc_proj_state_x_coord.empty();        //clears previous incremented projected state
    pspolicy->at(sp).inc_proj_state_y_coord.empty();        //clears previous incremented projected state
    pspolicy->at(sp).inc_proj_state_z_coord.empty();        //clears previous incremented projected state
    //pspolicy->at(sp).inc_proj_state.resize(3*(pP->ca_inc+2));
    double x_inc_movement;
    double y_inc_movement;
    double z_inc_movement;
    
    //calculates the incremented movement for each dimension
    x_inc_movement = (pspolicy->at(sp).projected_state.at(0) - pspolicy->at(sp).current_state.at(0))/pP->ca_inc;
    y_inc_movement = (pspolicy->at(sp).projected_state.at(1) - pspolicy->at(sp).current_state.at(1))/pP->ca_inc;
    z_inc_movement = (pspolicy->at(sp).projected_state.at(2) - pspolicy->at(sp).current_state.at(2))/pP->ca_inc;
    
    //puts the incremented projected state into a vector
    for (int inc=0; inc<pP->ca_inc; inc++){
        pspolicy->at(sp).inc_proj_state_x_coord.at(inc) = (pspolicy->at(sp).current_state.at(0) + inc*x_inc_movement);
        pspolicy->at(sp).inc_proj_state_y_coord.at(inc) = (pspolicy->at(sp).current_state.at(1) + inc*y_inc_movement);
        pspolicy->at(sp).inc_proj_state_z_coord.at(inc) = (pspolicy->at(sp).current_state.at(2) + inc*z_inc_movement);
    }
    assert(pspolicy->at(sp).inc_proj_state_x_coord.size()==pP->ca_inc);
    assert(pspolicy->at(sp).inc_proj_state_y_coord.size()==pP->ca_inc);
    assert(pspolicy->at(sp).inc_proj_state_z_coord.size()==pP->ca_inc);
}


/////////////////////////////////////////////////////////////////
//Calculates the distance between to agents
double Simulator::Get_Distance_To_Other_Agent(vector<Policy>* pspolicy, int sp, int spp, double distance)
{
    double x;
    double x_mag;
    double y;
    double y_mag;
    double z;
    double z_mag;
    double r;
    
    x = pspolicy->at(sp).current_state.at(0)-pspolicy->at(spp).current_state.at(0);
    y = pspolicy->at(sp).current_state.at(1)-pspolicy->at(spp).current_state.at(1);
    z = pspolicy->at(sp).current_state.at(2)-pspolicy->at(spp).current_state.at(2);
    x_mag = x*x;
    y_mag = y*y;
    z_mag = z*z;
    r = x_mag+y_mag+z_mag;
    return r;
}


/////////////////////////////////////////////////////////////////
//Compares two agents incremented projected states
void Simulator::Compare_Agents_Inc_Projected_State(vector<Policy>* pspolicy, int sp, int spp)
{
    for (int inc=0; inc < pP->ca_inc; inc++)
    {
        double x;
        double x_mag;
        double y;
        double y_mag;
        double z;
        double z_mag;
        double r;
        //similarly this can be and should be written better for debugging purposes
        x = pspolicy->at(sp).inc_proj_state_x_coord.at(inc)-pspolicy->at(spp).inc_proj_state_x_coord.at(inc);
        y = pspolicy->at(sp).inc_proj_state_y_coord.at(inc)-pspolicy->at(spp).inc_proj_state_y_coord.at(inc);
        z = pspolicy->at(sp).inc_proj_state_z_coord.at(inc)-pspolicy->at(spp).inc_proj_state_z_coord.at(inc);
        x_mag = x*x;
        y_mag = y*y;
        z_mag = z*z;
        r = x_mag+y_mag+z_mag;
        //cout << r << endl;
        if (r <= (pP->ca_radius+pP->ca_radius))
        {
            //cout << "collision decteded" << endl;
            pspolicy->at(sp).current_speed = pP->ca_flight_speed;
            break;
        }
        else
        {
            //cout << "pass" << endl;
            pspolicy->at(sp).current_speed = pP->max_flight_velocity;
        }
    }
}


/////////////////////////////////////////////////////////////////
//Updates the conflict counter for an agent
void Simulator::Update_Conflict_Counter(vector<Policy>* pspolicy, int sp, int spp){
    //int agent_ID_1 = pspolicy->at(sp).team_ID;
    int agent_ID_2 = pspolicy->at(spp).team_ID;
    pspolicy->at(sp).conflict_counter.at(agent_ID_2) += 1;
}


/////////////////////////////////////////////////////////////////
//Runs the Collision check process
void Simulator::Check_for_Collisions(vector<Policy>* pspolicy){
    for (int sp=0; sp< pspolicy->size(); sp++)
    {
        //only considers agents who have not reached their final destination
        if (pspolicy->at(sp).at_final_waypoint == false)
        {
            for (int spp=0; spp < pspolicy->size(); spp++)
            {
                //only considers agent who have not reached their final destination
                if (pspolicy->at(spp).at_final_waypoint == false)
                {
                    if (sp!=spp)
                    {
                        double distance = 0;
                        distance = Get_Distance_To_Other_Agent(pspolicy, sp, spp, distance);
                        //cout << distance << endl;
                        //4*(2*pP->max_travel_dist+2*pP->ca_radius)
                        //multiplying by 4 might be a little over kill. we need to verify that this is working closer to the minimum distance that two agents need to be from one another to cause a potential conflict
                        if (distance<=4*(2*pP->max_travel_dist+2*pP->ca_radius))
                        {
                            Compare_Agents_Inc_Projected_State(pspolicy, sp, spp);
                            if (pspolicy->at(sp).current_speed == pP->ca_flight_speed)
                            {
                                //cout << "Potential Collision Between \t" << sp << "\t and \t" << spp << endl;
                                Update_Conflict_Counter(pspolicy, sp, spp);
                                Get_Projected_State(pspolicy, sp);
                            }
                        }
                        else
                        {
                            continue;
                        }
                    }
                    else
                    {
                        continue;
                    }
                }
                else
                {
                    continue;
                }
            }
        }
        else
        {
            continue;
        }
    }
}


/////////////////////////////////////////////////////////////////
//Runs the collision detection process
void Simulator::Collision_Detection(vector<Policy>* pspolicy){
    for (int sp=0; sp<pspolicy->size(); sp++){
        //only considers agent who have not reached their final destination
        if (pspolicy->at(sp).at_final_waypoint == false){
            pspolicy->at(sp).current_speed = pP->max_flight_velocity;
            pspolicy->at(sp).projected_state.empty();
            //gets the distance from the current state to the target waypoint
            Get_Dist_To_Target_Waypoint(pspolicy, sp);
            //gets the projected state
            Get_Projected_State(pspolicy, sp);
            //gets the incremented projected state
            Get_Inc_Projected_State(pspolicy, sp);
        }
    }
    Check_for_Collisions(pspolicy);
}


/////////////////////////////////////////////////////////////////
//Checks if each agent have reached their target waypoint
void Simulator::Check_If_At_Target_Waypoint(vector<Policy>* pspolicy, int sp){
    int tw = pspolicy->at(sp).target_waypoint;
    if (pspolicy->at(sp).current_state == pspolicy->at(sp).waypoint.at(tw).coordinate){
        if (tw < pP->num_waypoints-1){
            tw += 1;
        }
        else{
            //cout << "Reached Final Waypoint" << endl;
        }
    }
}


/////////////////////////////////////////////////////////////////
//Sets the current state for each agent to their projected state
void Simulator::Set_Current_State(vector<Policy>* pspolicy){
    for (int sp=0; sp<pspolicy->size(); sp++){
        if (pspolicy->at(sp).at_final_waypoint == false){
            pspolicy->at(sp).current_state = pspolicy->at(sp).projected_state;
            Check_If_At_Target_Waypoint(pspolicy, sp);
            pspolicy->at(sp).state_history.push_back(pspolicy->at(sp).current_state);
        }
    }
}


/////////////////////////////////////////////////////////////////
//Checks if each agent has reached their final waypoint
void Simulator::Check_If_At_Final_Waypoint(vector<Policy>* pspolicy){
    for (int sp=0; sp<pspolicy->size(); sp++){
       if (pspolicy->at(sp).at_final_waypoint == false){
           if (pspolicy->at(sp).target_waypoint == pP->num_waypoints-1){
               int tw = pspolicy->at(sp).target_waypoint;
               if (pspolicy->at(sp).current_state ==  pspolicy->at(sp).waypoint.at(tw).coordinate){
                   pspolicy->at(sp).at_final_waypoint = true;
               }
           }
       }
    }
}


/////////////////////////////////////////////////////////////////
//Runs the entire simulation process
void Simulator::Run_Simulation(vector<Policy>* pspolicy){
    //cout << "cp" << endl;
    Init_pspolicy(pspolicy);
    double ct = 0;
    while (ct < pP->time_max){
        Collision_Detection(pspolicy);
        Set_Current_State(pspolicy);
        Check_If_At_Final_Waypoint(pspolicy);
        ct += pP->delta_t;
    }
}

#endif /* Simulator_hpp */
