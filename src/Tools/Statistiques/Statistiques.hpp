#ifndef _Statistiques_
#define _Statistiques_

#include <iostream>
#include <cstring>
#include <chrono>
#include <vector>
#include <cstdio>
#include <cstdlib>

class Statistiques {
private:
    std::chrono::steady_clock::time_point start;

public:
    int nbTramesDetectees;
    int nbBonsCRCs;
    int nbBonsCRC_init;
    int nbBonsCRCs_1x;
    int nbBonsCRCs_2x;
    int nbBonsCRCs_3x;
    int nbDF18Frames;
    int nbStrangeFrames;

    Statistiques( );
    void dump();
    ~Statistiques( );

    /////////////////////////////////////////////////////////////////////////////

    void add_detected_frame();
    void validated_crc_init();
    void validated_crc_brute_1x();
    void validated_crc_brute_2x();
    void validated_crc_brute_3x();
    void add_type_18_frame();
    void add_strange_frame();

    int  validated_crc  ();
    int  detected_frames();

    /////////////////////////////////////////////////////////////////////////////
};

#endif