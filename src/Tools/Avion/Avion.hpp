#ifndef _Avion_
#define _Avion_

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

class Avion {
private:
    float altitude;

    float speed_h;      // vitesse horizontale
    float speed_v;      // vitesse verticale
    float angle;        // angle de l'avion

    float dist_min;     // distance minimum mesurée
    float dist_max;     // distance maximum mesurée
    float dist_curr;    // distance actuelle de l'avion

    int32_t OACI;       // identifiant hexadecimal
    int32_t type;       // type de l'avion
    char name[9];       // nom de l'avion
    bool GNSS;          // informations GNSS ?

    int32_t updates;    // nombre de messages recus

    std::chrono::time_point<std::chrono::system_clock> lastUpdate;
    bool modified;

public:
    std::vector<float>  list_long;
    std::vector<float>  list_lat;
    std::vector<int8_t> list_crc; // validated with/without brute force

    float lon_odd;      // ODD and EVEN positions
    float lat_odd;      // of the planes used to
    float lon_even;     // precisely compute their
    float lat_even;     // positions

    std::vector<int32_t> list_altitude;

    float last_score;
    float mini_score;
    float maxi_score;

public:

    Avion(const int32_t _OACI);

    /////////////////////////////////////////////////////////////////////////////

    void set_GNSS_mode(const bool value);

    /////////////////////////////////////////////////////////////////////////////

    void update_distance();

    /////////////////////////////////////////////////////////////////////////////

    void set_type(const int32_t value);
    int  get_type();

    /////////////////////////////////////////////////////////////////////////////

    void  set_latitude(const float value);
    float get_latitude();

    /////////////////////////////////////////////////////////////////////////////

    void  set_longitude(const float value);
    float get_longitude();

    /////////////////////////////////////////////////////////////////////////////

    void  set_reliability(const int value);
    int   get_reliability();

    /////////////////////////////////////////////////////////////////////////////

    void  set_score(const float value);
    float get_score();
    float get_min_score();
    float get_max_score();

    /////////////////////////////////////////////////////////////////////////////

    void  set_speed_horizontal(const float value);
    float get_speed_horizontal();

    /////////////////////////////////////////////////////////////////////////////

    void  set_speed_vertical(const float value);
    float get_speed_vertical();

    /////////////////////////////////////////////////////////////////////////////

    void  set_angle(const float value);
    float get_angle();

    /////////////////////////////////////////////////////////////////////////////

    void  set_altitude(const float value);
    float get_altitude();

    /////////////////////////////////////////////////////////////////////////////

    float get_dist_cur();
    float get_dist_min();
    float get_dist_max();

    /////////////////////////////////////////////////////////////////////////////

    int32_t get_OACI();

    /////////////////////////////////////////////////////////////////////////////

    void  set_name(const char *value);
    char* get_name();

    /////////////////////////////////////////////////////////////////////////////

    int32_t get_messages();

    /////////////////////////////////////////////////////////////////////////////

    void      update();
    int  last_update();

    /////////////////////////////////////////////////////////////////////////////

    void print();
    void store(FILE* file);

    /////////////////////////////////////////////////////////////////////////////
};

#endif