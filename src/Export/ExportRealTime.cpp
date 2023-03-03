#include "ExportRealTime.hpp"
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
ExportRealTime::ExportRealTime(const bool _enable)
{
    enable = _enable;
}
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
void ExportRealTime::update( std::vector<Avion*>& liste_v )
{
    if( enable == false )   // Si le module est inactif alors on
        return;             // ne fait rien !


    FILE* rt = fopen( "/tmp/avions.gps", "w" );
    if( rt == NULL )
    {
        printf("(EE) An error occurs when openning the /tmp/avions.gps file !\n");
        exit( EXIT_FAILURE );
    }


    const int nPlanes = liste_v.size();
    for (int p = 0; p < nPlanes; p += 1)
    {
        if( (liste_v.at(p)->last_update() > 600) || (liste_v.at(p)->get_messages() == 1) )
            continue;

        const int nLon = liste_v.at(p)->list_long.size();
        const int oaci = liste_v.at(p)->get_OACI();
        const int type = liste_v.at(p)->get_type();

        for (int t = 0; t < nLon; t += 1)
        {
            const float lon = liste_v.at(p)->list_long.at(t);
            const float lat = liste_v.at(p)->list_lat.at (t);
            const int   crc = liste_v.at(p)->list_crc.at (t); // validated or not validated
            fprintf(rt, "%+24.22f %+24.22f %d %06X %d %d\n", lat, lon, crc, oaci, type, p);
        }
    }
    fclose( rt );
}
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
void ExportRealTime::update_all( std::vector<Avion*>& liste_v )
{
    FILE* rt = fopen( "/tmp/avions.gps", "w" );
    if( rt == NULL )
    {
        printf("(EE) An error occurs when openning the /tmp/avions.gps file !\n");
        exit( EXIT_FAILURE );
    }


    const int nPlanes = liste_v.size();
    for (int p = 0; p < nPlanes; p += 1)
    {
        if( liste_v.at(p)->get_messages() == 1 )
            continue;

        const int nLon = liste_v.at(p)->list_long.size();
        const int oaci = liste_v.at(p)->get_OACI();
        const int type = liste_v.at(p)->get_type();

        for (int t = 0; t < nLon; t += 1)
        {
            const float lon = liste_v.at(p)->list_long.at(t);
            const float lat = liste_v.at(p)->list_lat.at (t);
            const int   crc = liste_v.at(p)->list_crc.at (t); // validated or not validated
            fprintf(rt, "%+24.22f %+24.22f %d %06X %d %d\n", lat, lon, crc, oaci, type, p);
        }
    }
    fclose( rt );
}
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
