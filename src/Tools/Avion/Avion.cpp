#include "Avion.hpp"
#include "../colors.hpp"
#include "type_aircraft.hpp"


inline long double toRadians(const long double degree) {
    long double one_deg = (M_PI) / 180;
    return (one_deg * degree);
}


inline long double distance(long double lat1, long double long1,
                            long double lat2, long double long2) {
    lat1  = toRadians(lat1);
    long1 = toRadians(long1);
    lat2  = toRadians(lat2);
    long2 = toRadians(long2);

    // Haversine Formula
    long double dlong = long2 - long1;
    long double dlat = lat2 - lat1;

    long double ans = pow(sin(dlat / 2), 2) +
                      cos(lat1) * cos(lat2) *
                      pow(sin(dlong / 2), 2);

    ans = 2 * asin(sqrt(ans));

    // Radius of Earth in
    // Kilometers, R = 6371
    // Use R = 3956 for miles
    long double R = 6371;

    // Calculate the result
    ans = ans * R;

    return ans;
}


Avion::Avion(const int32_t _OACI)
{
    speed_h   = 0.0f;
    speed_v   = 0.0f;
    angle     = 0.0f;

    dist_min  = 0.0f;
    dist_max  = 0.0f;
    dist_curr = 0.0f;

    OACI      = _OACI;
    updates   = 0;
    altitude  = 0.0f;
    modified  = false;
    GNSS      = false;
    type      = 0;       // Le type de l'avion ou du moins de l'objet volant !

    mini_score = 1.0f;
    maxi_score = 0.0f;
    last_score = 0.0f;

    for (uint32_t i = 0; i < 8; i += 1)
        name[i] = '-';
    name[8] = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Avion::set_GNSS_mode(const bool value)
{
    GNSS = value;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Avion::update_distance()
{
    const float lat = get_latitude();
    const float lon = get_longitude();
    dist_curr       = distance(lat, lon, 44.820783, -0.501887);
    if( dist_max != 0 ){
        dist_max        = ( dist_max > dist_curr ) ? dist_max : dist_curr;
        dist_min        = ( dist_min < dist_curr ) ? dist_min : dist_curr;
    }else{
        dist_max        = dist_curr;
        dist_min        = dist_curr;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int32_t Avion::get_type()
{
    return type;
}

void Avion::set_type(const int32_t value)
{
    type = value;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float Avion::get_latitude()
{
    if( list_lat.size() != 0 )
        return list_lat.at( list_lat.size() - 1 );
    else
        return 0.0f;
}

void Avion::set_latitude(const float value)
{
    list_lat.push_back( value );
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float Avion::get_longitude()
{
    if( list_long.size() != 0 )
        return list_long.at( list_long.size() - 1 );
    else
        return 0.0f;
}

void Avion::set_longitude(const float value)
{
    list_long.push_back( value );
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float Avion::get_score()
{
    return last_score;
}

float Avion::get_min_score()
{
    return mini_score;
}

float Avion::get_max_score()
{
    return maxi_score;
}

void Avion::set_score(const float value)
{
    mini_score = std::min(mini_score, value);
    maxi_score = std::max(maxi_score, value);
    last_score = value;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float Avion::get_speed_horizontal()
{
    return speed_h;
}

void Avion::set_speed_horizontal(const float value)
{
    speed_h = value;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float Avion::get_speed_vertical()
{
    return speed_v;
}

void Avion::set_speed_vertical(const float value)
{
    speed_v = value;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float Avion::get_angle()
{
    return angle;
}

void Avion::set_angle(const float value)
{
    angle = value;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float Avion::get_altitude()
{
    if( list_altitude.size() != 0 )
        return list_altitude.at( list_altitude.size() - 1 );
    else
        return 0.0f;
}

void Avion::set_altitude(const float value)
{
    list_altitude.push_back( value );
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float Avion::get_dist_cur()
{
    return dist_curr;
}

float Avion::get_dist_min()
{
    return dist_min;
}

float Avion::get_dist_max()
{
    return dist_max;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int32_t Avion::get_OACI()
{
    return OACI;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

char* Avion::get_name() {
    return name;
}

void Avion::set_name(const char *value) {
    strcpy(name, value);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int32_t Avion::get_messages()
{
    return updates;
}

void Avion::update()
{
    lastUpdate = std::chrono::system_clock::now();
    updates += 1;
    modified = true;
}

int32_t Avion::last_update()
{
    const auto curr   = std::chrono::system_clock::now();
    const int32_t seconds = std::chrono::duration_cast<std::chrono::seconds>(curr - lastUpdate).count();
    return seconds;
}

void Avion::print()
{
    if( modified ) green();
    printf("%06X | %s | %s | ", get_OACI(), toCodeName(get_type()), get_name());
    printf("%1.2f [%1.2f, %1.2f] | ", get_score(), get_min_score(), get_max_score());

    if(get_latitude() != 0) printf("%9.6f | %9.6f | ", get_latitude(), get_longitude());
    else                    printf("--------- | --------- | ");

    printf("%4d km/h | %4d m/mn | %4d° | ", (int32_t) get_speed_horizontal(),
           (int32_t) get_speed_vertical(), (int32_t) get_angle());

    if( GNSS ) printf("GNSS | ");
    else       printf("BARO | ");

    if( get_altitude() != 0 ) printf("%5d pds | ",   (int32_t) get_altitude());
    else                      printf("--------- | ");

    printf("%5d km [%3d,%3d] | ", (int32_t) get_dist_cur(), (int32_t) get_dist_min(), (int32_t) get_dist_max());
    printf("%6d | ", get_messages());

    const int32_t seconds = last_update();
    if( seconds > 60 ) printf("%5d mn |\n", seconds/60);
    else               printf("%6d s |\n",  seconds);

    if( modified ) black();
    modified = false;
}

void Avion::store(FILE* file)
{
    fprintf(file, "[PLANE] %06X (type = %s, name = %s )\n", get_OACI(), toCodeName(get_type()), get_name());
    //printf("%06X | %s | %s | ", get_OACI(), toCodeName(get_type()), get_name());
    fprintf(file, " - last score %1.2f [min = %1.2f, max = %1.2f]\n", get_score(), get_min_score(), get_max_score());
    if( GNSS ) fprintf(file, " - plane sensor : GNSS\n");
    else       fprintf(file, " - plane sensor : BARO\n");
    fprintf(file, " - last position %9.6f | %9.6f\n", get_latitude(), get_longitude());
    if(get_latitude() != 0) fprintf(file, " - last position %9.6f | %9.6f\n", get_latitude(), get_longitude());
    else                    fprintf(file, " - last position --------- | ---------\n");
    if(get_altitude() != 0) fprintf(file, " - last altitude %5d pds\n",   (int32_t) get_altitude());
    else                    fprintf(file, " - last altitude ---------\n");
    fprintf(file, " - others %4d km/h | %4d m/mn | %4d°\n", (int32_t) get_speed_horizontal(), (int32_t) get_speed_vertical(), (int32_t) get_angle());

    fprintf(file, " - last distance %5d km [min = %3d km, max = %3d km]\n", (int32_t) get_dist_cur(), (int32_t) get_dist_min(), (int32_t) get_dist_max());
    fprintf(file, " - #messages %6d\n", get_messages());
    fprintf(file, "                     latitude |                  longitude |    altitude\n");
    for (uint32_t jj = 0; jj < list_lat.size(); jj += 1)    // pour tous les positions
    {
        fprintf(file, "   %24.22f  | %24.22f  | %8d pds\n", list_lat[jj], list_long[jj], list_altitude[jj]);
    }
    fprintf(file, "\n");
    fprintf(file, "\n");
}
