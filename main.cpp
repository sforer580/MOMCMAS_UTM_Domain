//
//  main.cpp
//  MCMAS_UTM_DOMAIN
//
//  Created by Scott S Forer on 3/31/18.
//  Copyright © 2018 MCMAS_UTM_DOMAIN. All rights reserved.
//

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

#include "Parameters.hpp"
#include "Waypoint.hpp"
#include "Policy.hpp"
#include "Agent.hpp"
#include "Team.hpp"
#include "Simulator.hpp"
#include "CCEA.hpp"

int main() {
    srand(time(NULL));
    Parameters P;
    P.Init_Parameters();
    CCEA CEA;
    CEA.pP = &P;
    CEA.Run_Program();

}