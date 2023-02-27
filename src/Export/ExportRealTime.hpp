#ifndef _ExportRealTime_
#define _ExportRealTime_

#include <iostream>
#include <cmath>
#include <complex>
#include <cstring>
#include <chrono>
#include <bitset>
#include <vector>
#include <fstream>
#include <cstdio>
#include <cstdlib>

#include "../Tools/Avion/Avion.hpp"

using namespace std;

class ExportRealTime
{
private:
    bool enable;

public:
    ExportRealTime(const bool _enable);

    void update( std::vector<Avion*>& liste_v );
};

#endif