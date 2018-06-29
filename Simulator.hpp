//
//  Simulator.hpp
//  MCMAS_UTM_DOMAIN
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
    void Init_Conflict_Counter(int sp, vector<Policy>* pspolicy);
    void Init_Flight_Speed(int sp, vector<Policy>* pspolicy);
    void Init_Target_Waypoint(int sp, vector<Policy>* pspolicy);
    void Init_Final_Waypoint_Statement(int sp, vector<Policy>* pspolicy);
    void Init_Current_State(int sp, vector<Policy>* pspolicy);
    void Init_pspolicy(vector<Policy>* pspolicy);
    void Run_Simulation(vector<Policy>* pspolicy);
    
    //
    void Get_Dist_To_Target_Waypoint(vector<Policy>* pspolicy, int sp);
    void Calc_Projected_State(vector<Policy>* pspolicy, int sp);
    void Get_Projected_State(vector<Policy>* pspolicy, int sp);
    void Get_Inc_Projected_State(vector<Policy>* pspolicy, int sp);
    double Get_Distance_To_Other_Agent(vector<Policy>* pspolicy, int sp, int spp, double distance);
    void Compare_Agents_Projected_State(vector<Policy>* pspolicy, int sp, int spp);
    void Run_Conflict_Counter(vector<Policy>* sim_team, int sim_p, int sim_pp);
    void Check_for_Collisions(vector<Policy>* pspolicy);
    void Crash_Avoidance(vector<Policy>* pspolicy);
    
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
//Initializes the conflict counter for each policy
void Simulator::Init_Conflict_Counter(int sp, vector<Policy>* pspolicy){
    for (int t=0; t<pP->num_teams; t++){
        pspolicy->at(sp).conflict_counter.at(t) = 0;
    }
}


/////////////////////////////////////////////////////////////////
//Initializes the flight speed for each policy
void Simulator::Init_Flight_Speed(int sp, vector<Policy>* pspolicy){
    for (int t=0; t<pP->num_teams; t++){
        pspolicy->at(sp).current_speed = pP->max_flight_velocity;
    }
}


/////////////////////////////////////////////////////////////////
//Initializes the target waypoint for each policy
void Simulator::Init_Target_Waypoint(int sp, vector<Policy>* pspolicy){
    for (int t=0; t<pP->num_teams; t++){
        pspolicy->at(sp).target_waypoint = 1;
    }
}


/////////////////////////////////////////////////////////////////
//Initializes the final waypoint statement for each policy
void Simulator::Init_Final_Waypoint_Statement(int sp, vector<Policy>* pspolicy){
    for (int t=0; t<pP->num_teams; t++){
        pspolicy->at(sp).at_final_waypoint = false;
    }
}


/////////////////////////////////////////////////////////////////
//Initializes the current state for each policy
void Simulator::Init_Current_State(int sp, vector<Policy>* pspolicy){
    for (int t=0; t<pP->num_teams; t++){
        pspolicy->at(sp).current_state = pspolicy->at(sp).waypoint.at(0).coordinate;
        pspolicy->at(sp).state_history.push_back(pspolicy->at(sp).current_state);
    }
}


/////////////////////////////////////////////////////////////////
//Initializes the policies
void Simulator::Init_pspolicy(vector<Policy>* pspolicy){
    for (int sp=0; sp<pspolicy->size(); sp++){
        Init_Conflict_Counter(sp, pspolicy);
        Init_Flight_Speed(sp, pspolicy);
        Init_Target_Waypoint(sp, pspolicy);
        Init_Current_State(sp, pspolicy);
    }
}


/////////////////////////////////////////////////////////////////
//Calculate Distance to Target Waypoint
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
//Calculates New Projected State
//gets the projected state using an agents current travel speed
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
    travel_dist = pP->max_travel_dist;
    
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
    
    //Calculates current state
    current_x = pspolicy->at(sp).current_state.at(0) + travel_dist*D_1;
    current_y = pspolicy->at(sp).current_state.at(1) + travel_dist*D_2;
    current_z = pspolicy->at(sp).current_state.at(2) + travel_dist*D_3;
    pspolicy->at(sp).projected_state.empty();
    pspolicy->at(sp).projected_state.at(0) = current_x;
    pspolicy->at(sp).projected_state.at(1) = current_y;
    pspolicy->at(sp).projected_state.at(2) = current_z;
    //cout << "cp2" << endl;
}


/////////////////////////////////////////////////////////////////
//Calculates The Projected State
//gets the projected state of an agent if it were to travel at its max velocity
void Simulator::Get_Projected_State(vector<Policy>* pspolicy, int sp)
{
    //cout << "cp5" << endl;
    //check_if_at_waypoint(pp, jj, dist_to_target_waypoint, waypoint_telem, max_travel_dist, current_telem, target_waypoint, num_waypoints);
    
    if (pspolicy->at(sp).distance_to_target_waypoint > 0){
        if (pspolicy->at(sp).distance_to_target_waypoint < pP->max_travel_dist){
            //cout << "cp3" << endl;
            //checks to see if the distance from the current state to the target waypoint is within the max travel distance
            vector<double> proj_state = pspolicy->at(sp).waypoint.at(pspolicy->at(sp).target_waypoint).coordinate;
            pspolicy->at(sp).projected_state = proj_state;
        }
    }
    else if (pspolicy->at(sp).distance_to_target_waypoint > 0){
        if (pspolicy->at(sp).distance_to_target_waypoint > pP->max_travel_dist){
            //cout << "cp4" << endl;
            //calculates the new projected state
            Calc_Projected_State(pspolicy, sp);
        }
    }
    else{
        vector<double> proj_state = pspolicy->at(sp).waypoint.at(pspolicy->at(sp).target_waypoint).coordinate;
        pspolicy->at(sp).projected_state = proj_state;
    }
    //cout << "projected telem" << endl;
    //for (int ll=0; ll < 3; ll++)
    //{
    //cout << system.at(pp).agents.at(jj).projected_telem.at(ll) << "\t";
    //}
    //cout << endl;
}


/////////////////////////////////////////////////////////////////
//Calculates The Incremented Projected state
//gets the incremented projected state for each agent using a preset parameter
void Simulator::Get_Inc_Projected_State(vector<Policy>* pspolicy, int sp){
    pspolicy->at(sp).inc_proj_state.empty();        //clears previous incremented projected state
    //pspolicy->at(sp).inc_proj_state.resize(3*(pP->ca_inc+2));
    double x_inc_movement;
    double y_inc_movement;
    double z_inc_movement;
    
    //calculates the incremented movement for each dimension
    x_inc_movement = (pspolicy->at(sp).projected_state.at(0) - pspolicy->at(sp).current_state.at(0))/pP->ca_inc;
    y_inc_movement = (pspolicy->at(sp).projected_state.at(1) - pspolicy->at(sp).current_state.at(1))/pP->ca_inc;
    z_inc_movement = (pspolicy->at(sp).projected_state.at(2) - pspolicy->at(sp).current_state.at(2))/pP->ca_inc;
    
    //puts the incremented projected state into a vector
    for (int iii=0; iii<pP->ca_inc+2; iii++){
        //this could be changed to use three seperate vectors to make debugging easier. this did work in the previous version though
        pspolicy->at(sp).inc_proj_state.at(iii*3) = (pspolicy->at(sp).current_state.at(0) + iii*x_inc_movement);
        pspolicy->at(sp).inc_proj_state.at(iii*3+1) = (pspolicy->at(sp).current_state.at(1) + iii*y_inc_movement);
        pspolicy->at(sp).inc_proj_state.at(iii*3+2) = (pspolicy->at(sp).current_state.at(2) + iii*z_inc_movement);
    }
}


/////////////////////////////////////////////////////////////////
//Finds the distance of the current telems of the two agents in question
double Simulator::Get_Distance_To_Other_Agent(vector<Policy>* pspolicy, int sp, int spp, double distance)
{
    double x;
    double x_mag;
    double y;
    double y_mag;
    double z;
    double z_mag;
    double r;
    
    /*
     cout << "agent" << "\t" << sim_p << endl;
     cout << "current telem" << endl;
     for (int i=0; i<3; i++)
     {
     cout << pspolicy->at(sim_p).current_telem.at(i) << "\t";
     }
     cout << endl;
     cout << "agent" << "\t" << sim_pp << endl;
     cout << "current telem" << endl;
     for (int i=0; i<3; i++)
     {
     cout << pspolicy->at(sim_pp).current_telem.at(i) << "\t";
     }
     cout << endl;
     */
    
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
//Comapres Agents Incremented Projected state
//gets distnace from one agents projected state to another
void Simulator::Compare_Agents_Projected_State(vector<Policy>* pspolicy, int sp, int spp)
{
    for (int kk=0; kk < pP->ca_inc+2; kk++)
    {
        double x;
        double x_mag;
        double y;
        double y_mag;
        double z;
        double z_mag;
        double r;
        //similarly this can be and should be written better for debugging purposes
        x = pspolicy->at(sp).inc_proj_state.at(kk+(kk*2))-pspolicy->at(spp).inc_proj_state.at(kk+(kk*2));
        y = pspolicy->at(sp).inc_proj_state.at(kk+(kk*2)+1)-pspolicy->at(spp).inc_proj_state.at(kk+(kk*2)+1);
        z = pspolicy->at(sp).inc_proj_state.at(kk+(kk*2)+2)-pspolicy->at(spp).inc_proj_state.at(kk+(kk*2)+2);
        x_mag = x*x;
        y_mag = y*y;
        z_mag = z*z;
        r = x_mag+y_mag+z_mag;
        //cout << r << endl;
        if (r <= (pP->ca_radius+pP->ca_radius))
        {
            //cout << "collision decteded" << endl;
            pspolicy->at(sp).current_speed = pP->ca_flight_speed;
            //cout << system.at(ii).agents.at(jj).current_travel_speed << endl;
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
//Checks For Possible Collisions
//checks the distance of each agents projected state against one another
void Simulator::Run_Conflict_Counter(vector<Policy>* pspolicy, int sp, int spp){
    //int agent_ID_1 = pspolicy->at(sp).team_ID;
    int agent_ID_2 = pspolicy->at(spp).team_ID;
    pspolicy->at(sp).conflict_counter.at(agent_ID_2) += 1;
}


/////////////////////////////////////////////////////////////////
//Checks For Possible Collisions
//checks the distance of each agents projected state against one another
void Simulator::Check_for_Collisions(vector<Policy>* pspolicy){
    //cout << pspolicy->size() << endl;
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
                        //get distance to other agent
                        distance = Get_Distance_To_Other_Agent(pspolicy, sp, spp, distance);
                        //cout << distance << endl;
                        //4*(2*pP->max_travel_dist+2*pP->ca_radius)
                        //multiplying by 4 might be a little over kill. we need to verify that this is working closer to the minimum distance that two agents need to be from one another to cause a potential conflict
                        if (distance<=4*(2*pP->max_travel_dist+2*pP->ca_radius))
                        {
                            //cout << sp << "\t" << spp << endl;
                            Compare_Agents_Projected_State(pspolicy, sp, spp);
                            if (pspolicy->at(sp).current_speed == pP->ca_flight_speed)
                            {
                                Run_Conflict_Counter(pspolicy, sp, spp);
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
//Runs the entire crash avoidance process
void Simulator::Crash_Avoidance(vector<Policy>* pspolicy){
    for (int sp=0; sp<pspolicy->size(); sp++){
        //only considers agent who have not reached their final destination
        if (pspolicy->at(sp).at_final_waypoint == false){
            pspolicy->at(sp).current_speed = pP->max_flight_velocity;
            pspolicy->at(sp).projected_state.empty();
            //gets the distance from the current state to the target waypoint
            Get_Dist_To_Target_Waypoint(pspolicy, sp);
            //gets the projected state
            Get_Projected_State(pspolicy, sp);
            //cout << "cp1" << endl;
            //gets the incremented projected state
            Get_Inc_Projected_State(pspolicy, sp);
        }
    }
    Check_for_Collisions(pspolicy);
}


/////////////////////////////////////////////////////////////////
//Runs the entire simulation
void Simulator::Run_Simulation(vector<Policy>* pspolicy){
    //cout << "cp" << endl;
    Init_pspolicy(pspolicy);
    double ct = 0;
    while (ct < pP->time_max){
        Crash_Avoidance(pspolicy);
        ct += pP->delta_t;
    }
}

#endif /* Simulator_hpp */
