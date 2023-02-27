#pragma clang diagnostic push
#pragma ide diagnostic ignored "bugprone-reserved-identifier"

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
#include <getopt.h>
#include <signal.h>

#include "Tools/Avion/type_aircraft.hpp"

#include "Tools/Parameters/Parameters.hpp"
#include "Tools/CTickCounter/CTickCounter.hpp"

#include "Radio/Receiver/Library/ReceiverLibrary.hpp"

//
// Les classes utiles pour exporter les données vers d'autres outils
//

    #include "Export/ExportRealTime.hpp"

//
//  CplxModule des nombres complexes => module flottant
//

#include "Processing/CplxModule/Library/CplxModuleLibrary.hpp"
#include "Processing/Detector/Library/DetectorLibrary.hpp"
#include "Processing/ADSBSynchro/RemoveADSBSynchro.hpp"
#include "Processing/DataPacking/BitPacking.hpp"
#include "Processing/CRC32b/CheckCRC32b/CheckCRC32b.hpp"
#include "Processing/CRC32b/RemoveCRC32b/RemoveCRC32b.hpp"


//
//  Correlateur permettant de détecter le prologue des trames ADSB
//

//#include "../../src/Frame/Frame.hpp"
#include "Processing/Sampling/DownSampling.hpp"
#include "Processing/PPM/PPM_demod.hpp"

#include "Tools/colors.hpp"

#include "tools.hpp"
//#include "ADSBFrame.hpp"
#include "Tools/Avion/Avion.hpp"

using namespace std;

bool isFinished = false;

void my_ctrl_c_handler(int s) {
    if (isFinished == true) {
        exit(EXIT_FAILURE);
    }
    isFinished = true;
}


struct LLR
{
    int index;
    float    value;
};

bool compareLLRs(const LLR a, const LLR b)
{
    return a.value < b.value;
}


/*   ============================== MAIN =========================== */
/*
	4 ech = 1 symb
	1 trame = 120 symb = 480 ech
*/


#include "brute_force/brute_force_1x.hpp"
#include "brute_force/brute_force_2x.hpp"
#include "brute_force/brute_force_3x.hpp"

// try_brute_force_1x


const int max_level = 8;
bool try_brute_force_llrs(
        std::vector<LLR> vec_score,
        vector<uint8_t>& vec_pack,
        vector<uint8_t>& vec_demod,
        vector<uint8_t>& vec_sync,
        int start, int level)
{
    if( level >= max_level )
        return false;

    for (int j = start; j < max_level; j += 1)
    {
//        printf("Level %d - Start %d] Iter = %d, Switching bit %d\n", level, start, j, vec_score[j].index);
        flipbit( vec_pack.data(), vec_score[j].index ); // On flip le bit le moins fiable

        bool crc_is_ok = check_crc<112 / 8>( vec_pack.data() );
        if (crc_is_ok == true){
            vec_demod[vec_score[j].index + 8] = vec_demod[vec_score[j].index + 8];       // On inverse le bit
            vec_sync [vec_score[j].index    ] = !vec_sync[vec_score[j].index    ];       // On inverse le bit
            return true;
        }else{
            bool result = try_brute_force_llrs(vec_score, vec_pack, vec_demod, vec_sync, j+1, level+1);
            if( result == true )
            {
                vec_demod[vec_score[j].index + 8] = vec_demod[vec_score[j].index + 8];       // On inverse le bit
                vec_sync [vec_score[j].index    ] = !vec_sync[vec_score[j].index    ];       // On inverse le bit
                return true;
            }
        }
        flipbit( vec_pack.data(), vec_score[j].index );
    }
    return false;
}


bool try_brute_force_llrs(vector<uint8_t>& vec_pack, vector<uint8_t>& vec_demod, vector<uint8_t>& vec_sync, vector<float>& vec_down)
{
    bool crc_brute_llr = false;
    //
    // On calcule la difference les "LLRs"
    //
    std::vector<LLR> vec_score (112);
    for(int32_t ii = 16; ii < 240; ii += 2)
    {
        const float diff = vec_down[ii] - vec_down[ii + 1];
        LLR llr;
        llr.value = std::abs(diff);
        llr.index = (ii / 2) - 8;
        vec_score[(ii / 2) - 8] = llr;
    }

    //
    // On trie les LLRs en fonction de leurs probabilités
    //
    sort(vec_score.begin(), vec_score.end(), compareLLRs);

    for(int32_t ii = 0; ii < 6; ii += 1)
        printf("%3d : [index = %3d, value = %17.13f]\n", ii, vec_score[ii].index, vec_score[ii].value);

    bool resu = try_brute_force_llrs( vec_score, vec_pack, vec_demod, vec_sync, 0, 0);
//    exit( 0 );
    return resu;
}




int main(int argc, char *argv[])
{
    printf("==================================== ADSB ====================================");
    printf("par Florian LOUPIAS - Février 2020\n"         );
    printf("par Bertrand LE GAL - Octobre 2020 -> .....\n");
    printf("==================================== ADSB ====================================");

    //
    // Precomputing the CRC tables to speed-up the computations
    // during frame decoding...
    //

    init_crc_lut();

    //
    // Default GPS values for the receiver position. They are used to compute the
    // plane distances.
    //

    float ref_latitude   = 44.820919; // latitude  de la maison
    float ref_longitude  = -0.502448; // longitude de la maison

    //
    // Creating the default objects used to store the planes after the decoding process
    //

    std::map<int32_t, Avion*> liste_m;
    std::vector<Avion *> liste_v;

    //
    // Initializing the CTRL+C handler used to stop the application
    //

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_ctrl_c_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    //
    // Initialisation of various counter used to profile
    // performances => in a specific class ?
    //

    int buffer_size = 65536;

    int nbTramesDetectees = 0; // nbre trame ...
    int nbBonsCRCs      = 0;
    int nbBonsCRC_init  = 0;
    int nbBonsCRCs_1x   = 0;
    int nbBonsCRCs_2x   = 0;
    int nbBonsCRCs_3x   = 0;
    int nbBonsCRCs_llr  = 0;
    int nbDF18Frames    = 0;
    int nbStrangeFrames = 0;

    //
    // Creation an initialization of the parametres used to configure
    // the ADSB receiver.
    //

    Parameters param;

    param.set("mode_radio", "radio");
    param.set("filename", "usrp");

    param.set("fc", 1090000000.0);
    param.set("fe",  4000000.0);

    param.set("hackrf_amplifier", -1);

    param.set("receiver_gain", -1);

#if defined(__ARM_NEON)
    param.set("mode_conv", "NEON"); // scalar
    param.set("mode_corr", "NEON_Intra"); // scalar
#elif defined(__AVX2__)
    param.set("mode_conv", "AVX2"); // scalar
    param.set("mode_corr", "AVX2"); // scalar
#else
    param.set("mode_conv", "scalar"); // scalar
    param.set("mode_corr", "scalar"); // scalar
#endif
    
    param.set("payload", 60);

    param.set("verbose", 0);

    param.set("ps_min", 0.75f);

    param.set("dump_frame", false);

    param.set("crystal_correct", 0);

    param.set("list_mode", false);

    param.set("brute_force_1x", false);
    param.set("brute_force_2x", false);
    param.set("brute_force_3x", false);
    param.set("brute_force_llr",false);

    //
    // Definition of the parameter list used by get_long feature to parse command line values
    //

    static struct option long_options[] =
            {
                    {"verbose",     no_argument,       NULL, 'v'},  // affiche temps sur chaque boucle Np + cplx => abs
                    {"seuil",       required_argument, NULL, 's'},  // pour changer la valeur min de la correlation (synchro)
                    {"np",          required_argument, NULL, 'n'},  // pour changer le nombre de boucle Np (ie nbre echantillon*200000) // Np = 10 => 0.5 s
                    {"conv",        required_argument, NULL, 'c'}, // a partir d'un fichier
                    {"corr",        required_argument, NULL, 'd'}, // a partir d'un fichier
                    {"radio",       required_argument, NULL, 'r'}, // a partir d'un fichier
                    {"file",        required_argument, NULL, 'F'}, // a partir d'un fichier
                    {"file-stream", required_argument, NULL, 'Q'}, // a partir d'un fichier
                    {"fc",          required_argument, NULL, 'f'}, // changer la frequence de la porteuse
                    {"fe",          required_argument, NULL, 'e'}, // changer la frequence echantillonnage
                    {"payload",     required_argument, NULL, 'p'}, // changer la frequence echantillonnage
                    {"amplifier",   required_argument, NULL, 'A'}, // changer la frequence echantillonnage
                    {"rcv_gain",    required_argument, NULL, 'L'}, // changer la frequence echantillonnage
                    {"rcv-gain",    required_argument, NULL, 'L'}, // changer la frequence echantillonnage
                    {"brute-force-1x",  no_argument,   NULL, '1'}, // changer la frequence echantillonnage
                    {"brute-force-2x",  no_argument,   NULL, '2'}, // changer la frequence echantillonnage
                    {"brute-force-3x",  no_argument,   NULL, '3'}, // changer la frequence echantillonnage
                    {"brute-force-llr", no_argument,   NULL, 'X'}, // changer la frequence echantillonnage
                    {"inter",       no_argument,       NULL, 'I'}, // changer la frequence echantillonnage
                    {"intra",       no_argument,       NULL, 'i'}, // changer la frequence echantillonnage
                    {"dump-frames", no_argument,       NULL, 'D'}, // changer la frequence echantillonnage
                    {"ppm",         required_argument, NULL, 'P'}, // changer la frequence echantillonnage
                    {"liste",       no_argument,       NULL, 'l'}, // changer la frequence echantillonnage
                    {"buffer_size",     required_argument,       NULL, 'B'}, // changer la frequence echantillonnage

                    {"ENSEIRB",     no_argument,       NULL, 'E'}, // changer la frequence echantillonnage
                    {NULL, 0,                          NULL, 0}
            };


    //
    // Parsing command line values
    //

    int option_index = 0;
    int c; //getopt
    while ((c = getopt_long(argc, argv, "be:p:f:n:s:vt8", long_options, &option_index)) != -1) {
        switch (c) {
            case 0:
                printf("%soption %s%s", long_options[option_index].name, KNRM, KRED);
                if (optarg)
                    printf("%s with arg %s%s", optarg, KNRM, KRED);
                printf("\n");
                break;

            case 'f':
                param.set("fc", std::stod(optarg));
                break;

            case 'e' :
                param.set("fe", std::stoi(optarg));
                break;

            case 'A' :
                param.set("hackrf_amplifier", std::stoi(optarg));
                break;

            case 'L' :
                param.set("receiver_gain", std::stoi(optarg));
                break;

            case 'p':
                param.set("payload", std::atoi(optarg));
                break;

            case 'B':
                param.set("buffer_size", std::atoi(optarg));
                buffer_size = std::atoi(optarg);
                break;

            case 's':
                param.set("ps_min", std::stof(optarg));
                if ((param.toFloat("ps_min") > 1.0f) || (param.toFloat("ps_min") < 0.0f)) {
                    printf("erreur : --produit_scalaire (ps_min) doit etre compris entre 0 et 1\n");
                    exit(EXIT_FAILURE);
                }
                break;

            case 'v':
                param.set("verbose", param.toInt("verbose") + 1);
                break;

            case '?':
                break;

            case 'c':
                param.set("mode_conv", optarg);
                break;

            case 'd':
                param.set("mode_corr", optarg);
                break;

            case 'r':
                param.set("mode_radio", "radio");
                param.set("filename", optarg);
                break;

            case 'F':
                param.set("mode_radio", "file");
                param.set("filename", optarg);
                break;

            case 'Q':
                param.set("mode_radio", "file-stream");
                param.set("filename", optarg);
                break;

            case 'E':
                ref_latitude  = 44.806884; // latitude  de l'ENSEIRB
                ref_longitude = -0.606629; // longitude de l'ENSEIRB
                break;

            case '1':
                param.set("brute_force_1x", true);
                break;

            case '2':
                param.set("brute_force_2x", true);
                break;

            case '3':
                param.set("brute_force_3x", true);
                break;

            case 'X':
                param.set("brute_force_llr", true);
                break;

            case 'D':
                param.set("dump_frame", true);
                break;

            case 'l':
                param.set("list_mode", true);
                break;

            case 'P' :
                param.set("crystal_correct", std::atoi(optarg));
                printf("%soption crystal_correct = %d dB%s\n", KNRM, param.toInt("crystal_correct"), KRED);
                break;

            default:
                printf("?? getopt returned character code 0%o ??\n", c);
                exit(EXIT_FAILURE);
                break;
        }
    }

    printf("%s", KRED);
    if (optind < argc) {
        printf("non-option ARGV-elements: ");
        while (optind < argc) {
            printf("%s ", argv[optind++]);
        }
        printf("\n");
        exit(EXIT_FAILURE);
    }
    printf("%s", KNRM);
    cout << endl;

    //
    // Creation of the buffer between [SdR module => demodulator]
    //

    vector<complex<float> > buffer(buffer_size);

    //
    // Allocation of the SdR/file access driver
    //

    Receiver *radio = ReceiverLibrary::allocate(param);

    //
    // Allocation of the converter (IQ => module)
    //

    CplxModule *conv = CplxModuleLibrary::allocate(param);

    //
    // Allocation of the frame detector (SIMD/inter/intra/...)
    //

    Detector *detect = DetectorLibrary::allocate(param);

    printf("(II) Launching the emitter application :\n");
    printf("(II) -> Modulation frequency    : %4d MHz\n", (int) (param.toDouble("fc") / 1000000.0));
    printf("(II) -> Symbol sampling freq.   : %4d MHz\n", (int) (param.toDouble("fe") / 1000000.0));
    printf("(II)\n");

    const int32_t crystal_correct_ppm = param.toInt("crystal_correct");
    if (crystal_correct_ppm != 0) {
        double freq_hz = param.toDouble("fc");
        double sample_rate_hz = param.toDouble("fe");

        sample_rate_hz = (int) ((double) sample_rate_hz * (1000000.0 - crystal_correct_ppm) / 1000000.0 + 0.5);
        freq_hz = freq_hz * (1000000.0 - crystal_correct_ppm) / 1000000.0;

        param.set("fc", freq_hz);
        param.set("fe", sample_rate_hz);

        printf("(II) -> Corrected modulation frequency    : %4d MHz\n", (int) (param.toDouble("fc") / 1000000.0));
        printf("(II) -> Corrected symbol sampling freq.   : %4d MHz\n", (int) (param.toDouble("fe") / 1000000.0));
        printf("(II)\n");
    }

    printf("(II) HackRF module configuration :\n");
    printf("(II) -> HackRF antenna parameter   : enable\n");
    printf("(II) -> HackRF amplifier parameter : disable\n");
    printf("(II)\n");


    //
    // Selection du module de conversion employé dans le programme
    //
    printf("(II) Processing system implementation :\n");
    printf("(II) -> Conversion IQ => module : %s\n", param.toString("mode_conv").c_str());
    printf("(II) -> Correlation computation : %s\n", param.toString("mode_corr").c_str());
    printf("(II)\n");


    //
    // Selection du module de conversion employé dans le programme
    //

    radio->initialize();
    radio->start_engine();

    auto start = std::chrono::system_clock::now();


    //
    // Selection du module de conversion employé dans le programme
    //

    CTickCounter timer;

    const float ps_min         = param.toFloat("ps_min");
    const bool brute_force_1x  = param.toBool ("brute_force_1x");
    const bool brute_force_2x  = param.toBool ("brute_force_2x");
    const bool brute_force_3x  = param.toBool ("brute_force_3x");
    const bool brute_force_llr = param.toBool ("brute_force_llr");

    std::vector<float> buffer_abs;
    std::vector<float> buffer_detect;

    DownSampling down(2);
    PPM_demod ppd;

    std::vector<uint8_t> buff_6;
    std::vector<uint8_t> buff_7;


    vector<float>   vec_data (480);
    vector<float>   vec_down (240);
    vector<uint8_t> vec_demod(120);
    vector<uint8_t> vec_sync (112);
    vector<uint8_t> vec_pack ( 14);

    DownSampling      o_down(2);
    PPM_demod         o_ppm;
    RemoveADSBSynchro o_sync;
    BitPacking        o_pack;
    CheckCRC32b       o_ccrc;
    RemoveCRC32b      c_rcrc;


//    Decoder_chain* dec_chain;
//    dec_chain = new Decoder_ADBS_like_chain( F.size_frame() );

    std::vector<uint8_t> frame_buf(112);


#ifdef _TRACE_MODE_
    std::ofstream of( "dec_frames.txt" );
#endif

//#define _DUMP_ALL_SIGNALS_


#ifdef _DUMP_ALL_SIGNALS_
    std::vector<uint8_t> frame_status( buffer_detect.size() );
#endif

    const bool StoreDataSet = param.toBool("dump_frame");

    FILE* file_iq_raw     = nullptr; // Le fichier contenant les données I/Q
    FILE* file_frames_bin = nullptr; // Le fichier contenant les 112 bits correspondant aux frames
    FILE* file_frames_dec = nullptr; // Le fichier contenant les trames décodées
    FILE* file_planes     = nullptr; // Le fichier contenant les informations concernant les avions
    FILE* file_coords     = nullptr; // Le fichier contenant les informations concernant les avions

    if( StoreDataSet == true )
    {
        //file_iq_raw     = fopen( "file_iq_raw.cs8", "w" );
        //file_frames_bin = fopen( "file_frames_bin.txt", "w" );
        file_frames_dec = fopen( "file_frames_dec.txt", "w" );
        file_planes     = fopen( "file_planes.txt", "w" );
        file_coords     = fopen( "file_coords.m", "w" );
    }

    bool firstAcq = true;

//    int stream_ptr = 0;

    const int verbose = param.toInt("verbose");

    //
    // On cree les modules qui vont stocker les données sur le disque
    // dur au fur et à mesure de l'execution
    //

    ExportRealTime dump_relatime( true  );

#if 0
    vector<int32_t> histo(65536);
    for (int i = 0; i < histo.size(); i++) histo[i] = 0;
#endif

    printf("+---------------+--------+--------+-----+--------+-----+----------+-------------+------+--------+-----+-----+-----+---------+---------+-------+-----+\n");
    printf("| Frame | Acqui | Echan. |  Corr. |  DF |   AA   | FTC |    CS    | ALT (ft) | CPRF | LON (deg) | LAT (deg) | Dis | Speed.H | Speed.V | Angle | CRC |\n");
    printf("+---------------+--------+--------+-----+--------+-----+----------+-------------+------+--------+-----------+-----+---------+---------+-------+-----+\n");

    if( file_frames_dec != nullptr )
    {
        fprintf(file_frames_dec, "+---------------+--------+--------+-----+--------+-----+----------+-------------+------+--------+-----+-----+-----+---------+---------+-------+-----+\n");
        fprintf(file_frames_dec, "| Frame | Acqui | Echan. |  Corr. |  DF |   AA   | FTC |    CS    | ALT (ft) | CPRF | LON (deg) | LAT (deg) | Dis | Speed.H | Speed.V | Angle | CRC |\n");
        fprintf(file_frames_dec, "+---------------+--------+--------+-----+--------+-----+----------+-------------+------+--------+-----------+-----+---------+---------+-------+-----+\n");
    }

    const int acq_per_sec = 2 * param.toInt("fe") / (buffer.size() -vec_data.size());

    int acq_counter = 0;

    while (radio->alive() && (isFinished == false))
    {
//        auto startIter = std::chrono::system_clock::now();

        int coverage = vec_data.size();
        coverage = (firstAcq == true) ? 0 : coverage;
        firstAcq = false;

#if 0
        printf("(II) RADIO RECEPTION (stream = %8u | nSample = %8lu)\n", stream_ptr, buffer.size() - coverage);
#endif

//        stream_ptr += buffer.size() - coverage;
#if 0
        int caff = cnt%64;
        if( caff ==   0 ) printf("Reception .   \r");
        if( caff ==  16 ) printf("Reception ..  \r");
        if( caff ==  32 ) printf("Reception ... \r");
        if( caff ==  48 ) printf("Reception ....\r");
        fflush(stdout);
        cnt += 1;
#endif
        timer.start_loading();
        bool error_n = radio->reception(buffer, coverage);
        timer.stop_loading();

        if (error_n == false) // Cela signifie que l'on a rencontré une erreur lors de la
            continue;         // reception des echantillons

//        continue;

        //
        // CALCUL DU MODULE DES VOIES (I,Q)
        //
        timer.start_conversion();
        conv->execute(&buffer, &buffer_abs);
        timer.stop_conversion();

        //
        // ON LANCE LA FONCTION DE DETECTION SUR L'ENSEMBLE DU BUFFER
        //
        timer.start_detection();
        detect->execute(&buffer_abs, &buffer_detect);
        timer.stop_detection();


        const int dump_decoded_frame = true;
        const int dump_resume        = param.toBool("list_mode");

        //
        // ON VA MAINTENANT PARCOURIR LE TABLEAU DE SCORE POUR DETECTER LES TRAMES
        //
        const int length = (buffer_abs.size() - coverage);

        for (int k = 0; k < length; k += 1)
        {

            float score = buffer_detect[k];

            if (score > ps_min)
            {

                //
                // Le score de la position k depasse le score minimum fixé par l'utilisateur
                //

                timer.start_decoding();
                nbTramesDetectees += 1;    // On vient de detecter qqchose

                //
                // On stocke les données pour une utilisation ultérieure
                //
                if( file_iq_raw != nullptr )
                {
                    int8_t dSet[960];
                    float* iSet = (float*)(buffer.data() + k);
                    float maxv = 0;
                    for (int x = 0; x < 960; x += 1)
                        maxv = std::max(maxv, iSet[x]);  // On suppose que les données que l'on récupere sont  -127 <=> +127
                    for (int x = 0; x < 960; x += 1)
                        dSet[x] = (maxv <= 1.0f) ? 127.0f * iSet[x] : iSet[x];  // On suppose que les données que l'on récupere sont  -127 <=> +127
                    fwrite(dSet, sizeof(int8_t), 960, file_iq_raw);
                    for (int x = 0; x < 60; x += 1) // On rajoute un peu de data entre 2 trames
                        dSet[x] = (rand()%6) - 3;
                    fwrite(dSet, sizeof(int8_t), 60, file_iq_raw);
                }

                for (int x = 0; x < vec_data.size(); x += 1)    // On extrait les 120 bits (x2) du vecteur
                    vec_data[x] = buffer_abs[x + k];            // d'echantillons (modules complexes de IQ)

                o_down.execute(vec_data,  vec_down);    // 480 values => 240 values
                o_ppm.execute (vec_down,  vec_demod);       // 240 values => 120 bits
                o_sync.execute(vec_demod, vec_sync);        // 112 bits
                o_pack.execute(vec_sync,  vec_pack);     // 15 octets

                const bool crc_initial = check_crc  <112 / 8>( vec_pack.data() );

                //
                // On stocke les données pour une utilisation ultérieure
                //
                if( file_frames_bin != nullptr ) {
                    if(crc_initial ) fprintf(file_frames_bin, "%5d | OK | [", nbTramesDetectees);
                    else             fprintf(file_frames_bin, "%5d | KO | [", nbTramesDetectees);
                    for (int x = 0; x < vec_sync.size() - 1; x += 1)
                        fprintf(file_frames_bin, "%d,", vec_sync[x]);
                    fprintf(file_frames_bin, "%d]\n", vec_sync[vec_sync.size() - 1]);
                }

                bool crc_brute_1x = false;
                if ( (crc_initial == false) && (brute_force_1x == true) )
                {
                    crc_brute_1x = try_brute_force_1x( vec_pack, vec_demod, vec_sync );
                }

                bool crc_brute_2x = false;
                if ( (crc_initial == false) && (crc_brute_1x == false) && (brute_force_2x == true) )
                {
                    crc_brute_2x = try_brute_force_2x( vec_pack, vec_demod, vec_sync );
                }

                bool crc_brute_3x = false;
                if ( (crc_initial == false) && (crc_brute_1x == false) && (crc_brute_2x == false) && (brute_force_3x == true) )
                {
                    crc_brute_3x = try_brute_force_3x( vec_pack, vec_demod, vec_sync );
                }

                bool crc_brute_llr = false;
                if ( (crc_initial == false) && (crc_brute_1x == false) && (crc_brute_2x == false) && (brute_force_llr == true)  )
                {
                    crc_brute_llr = try_brute_force_llrs( vec_pack, vec_demod, vec_sync, vec_down );
                }

                const bool crc_is_ok  = ((crc_initial == true)  || (crc_brute_1x == true) || (crc_brute_2x == true) || (crc_brute_3x == true));
                nbBonsCRCs     += crc_is_ok;
                nbBonsCRC_init += (crc_initial   == true);
                nbBonsCRCs_1x  += (crc_brute_1x  == true);
                nbBonsCRCs_2x  += (crc_brute_2x  == true);
                nbBonsCRCs_3x  += (crc_brute_3x  == true);
                nbBonsCRCs_llr += (crc_brute_llr == true);

                if ((crc_is_ok == true) || (crc_is_ok == false && verbose >= 2))
                {
                    const char* crc_show;
                         if( crc_initial   == true ) crc_show = "\x1B[32mOK\x1B[0m";
                    else if( crc_brute_1x  == true ) crc_show = "\x1B[33mOK\x1B[0m";
                    else if( crc_brute_2x  == true ) crc_show = "\x1B[31;1mOK\x1B[0m";
                    else if( crc_brute_3x  == true ) crc_show = "\x1B[31mOK\x1B[0m";
                    else if( crc_brute_llr == true ) crc_show = "\x1B[36mOK\x1B[0m";

                    if( verbose >= 1 )
                    {
                        printf("  [");
                        for(int32_t i = 0; i < vec_sync.size(); i += 1)
                            printf("%d ", vec_sync[i] );
                        printf("];\n");
                    }

                    if (crc_is_ok)
                    {

                        const int32_t df_value   = pack_bits(vec_sync.data(), 5);
                        const int32_t type_frame = pack_bits(vec_sync.data() + 32, 5);
                        const int32_t oaci_value = pack_bits(vec_sync.data() + 8, 24);

                        if( df_value == 18 ){
                            nbDF18Frames += 1;
                            continue;
                        }else if( df_value != 17 ){
                            nbStrangeFrames += 1;
                            continue;
                        }

                        Avion *ptr_avion = liste_m[oaci_value];
                        if (ptr_avion == nullptr) {
                            ptr_avion = new Avion(oaci_value);
                            liste_m[oaci_value] = ptr_avion;
                            liste_v.push_back(ptr_avion);
                        }
                        ptr_avion->update();
                        ptr_avion->set_score( score );

                        if ((type_frame >= 1) && (type_frame <= 4))
                        {
                            const int32_t type_airc    = pack_bits(vec_sync.data() + 37, 3);
                            const int32_t typeAricraft = toCode( type_frame, type_airc );
                            if( typeAricraft == -1 )
                            {
                                nbStrangeFrames += 1;
                                continue;
                            }

                            const char lut[] = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ##### ###############0123456789######";
                            char caractere[9];
                            caractere[0] = lut[ pack_bits(vec_sync.data() + 40, 6) ];
                            caractere[1] = lut[ pack_bits(vec_sync.data() + 46, 6) ];
                            caractere[2] = lut[ pack_bits(vec_sync.data() + 52, 6) ];
                            caractere[3] = lut[ pack_bits(vec_sync.data() + 58, 6) ];
                            caractere[4] = lut[ pack_bits(vec_sync.data() + 64, 6) ];
                            caractere[5] = lut[ pack_bits(vec_sync.data() + 70, 6) ];
                            caractere[6] = lut[ pack_bits(vec_sync.data() + 76, 6) ];
                            caractere[7] = lut[ pack_bits(vec_sync.data() + 82, 6) ];
                            caractere[8] = 0;

                            if (dump_decoded_frame && (dump_resume == false))
                                printf("| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d | %s |          |      |           |           |     |         |         |       |  %s |\n", nbBonsCRCs, acq_counter, k, score, df_value, oaci_value, type_frame, caractere, crc_show);
                            if( file_frames_dec != nullptr )
                                fprintf(file_frames_dec, "| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d | %s |          |      |           |           |     |         |         |       |  OK |\n", nbBonsCRCs, acq_counter, k, score, df_value, oaci_value, type_frame, caractere);

                            ptr_avion->set_type(typeAricraft);
                            ptr_avion->set_name(caractere);
                        }

                        if ((type_frame >= 5) && (type_frame <= 8) )
                        {
                            if (dump_decoded_frame && (dump_resume == false))
                                printf("| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |          |      |           |           |     |         |         |       |  %s |\n", nbBonsCRCs, acq_counter, k, score, df_value, oaci_value, type_frame, crc_show);
                            if( file_frames_dec != nullptr )
                                fprintf(file_frames_dec, "| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |          |      |           |           |     |         |         |       |  OK |\n", nbBonsCRCs, acq_counter, k, score, df_value, oaci_value, type_frame);
                        }

                        if ((type_frame >= 9) && (type_frame <= 18) )
                        {
                            const int32_t upper = pack_bits(vec_sync.data() + 40, 7);
                            const int32_t incr  = pack_bits(vec_sync.data() + 47, 1);
                            const int32_t lower = pack_bits(vec_sync.data() + 48, 4);
                            int32_t altitude = (upper << 4) | lower;
                            altitude = ( incr == 1) ? (altitude * 25 - 1000) : -1;

                            //
                            // On extrait les informations de la trame
                            //
                            const int32_t CPR_format = pack_bits(vec_sync.data() + 53, 1);
                            const float f_latitude   = pack_bits_float(vec_sync.data() + 54, 17);//(float)enc_latitude  / 131072.0; // divise par 2^17
                            const float f_longitude  = pack_bits_float(vec_sync.data() + 71, 17);//(float)enc_longitude / 131072.0; // divise par 2^17

//                            printf("%06X | CPR_format = %d\n", oaci_value, CPR_format);

                            //
                            // On calcule les parametres reels
                            //

                            float final_lat    = ComputeLatitude (f_latitude,  ref_latitude,                 CPR_format);
                            float final_lon    = ComputeLongitude(f_longitude, final_lat,     ref_longitude, CPR_format);
                            const int32_t dist = distance(final_lat, final_lon, ref_latitude, ref_longitude);


                            if (dump_decoded_frame && (dump_resume == false)) {
                                printf("| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |   %6d |    %d |  %8.5f |  %8.5f | %3d |         |         |       |  %s |\n", nbBonsCRCs, acq_counter, k, score, df_value, oaci_value, type_frame, (int32_t)altitude, CPR_format, final_lon, final_lat, dist, crc_show);
                            }
                            if( file_frames_dec != nullptr )
                                fprintf(file_frames_dec, "| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |   %6d |    %d |  %8.5f |  %8.5f | %3d |         |         |       |  OK |\n", nbBonsCRCs, acq_counter, k, score, df_value, oaci_value, type_frame, (int32_t)altitude, CPR_format, final_lon, final_lat, dist);

                            ptr_avion->set_altitude (altitude);
                            ptr_avion->set_GNSS_mode(false);
                            ptr_avion->set_latitude (final_lat);
                            ptr_avion->set_longitude(final_lon);
                            ptr_avion->update_distance();
                        }
#if 1
                        if (type_frame == 19)
                        {
                            if ((vec_demod[39 + 6] == 0) & (vec_demod[39 + 7] == 0) & (vec_demod[39 + 8] == 1)) {
                                // Type 1
                                // vitesse horizontale
                                int vNS = 0;
                                for (int q = 0; q < 10; q++) vNS += vec_demod[39 + 26 + q] * pow(2, 10 - q);
                                if (vec_demod[39 + 25] == 1) vNS = 1 - vNS;
                                else vNS = vNS - 1;
                                int vEW = 0;
                                for (int q = 0; q < 10; q++) vEW += vec_demod[39 + 15 + q] * pow(2, 10 - q);
                                if (vec_demod[39 + 25] == 1) vEW = 1 - vEW;
                                else vEW = vEW - 1;
                                float speed = sqrt(vEW * vEW + vNS * vNS);  // en kt ?? plutot en km/h
                                //speed = speed * 1.852;
                                // angle
                                float angle = atan2(vEW, vNS) * 360 / (2 * M_PI);
                                // vitesse verticale
                                int Vr = 0;
                                for (int q = 0; q < 8; q++) Vr += vec_demod[39 + 38 + q] * pow(2, 8 - q);
                                Vr = (Vr - 1) * 64;
                                if (vec_demod[39 + 37] == 1) Vr = -Vr; // en feet/min
                                Vr = Vr * 0.3048;
                                if (dump_decoded_frame && (dump_resume == false))
                                    printf("| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |          |      |           |           |     |    %4d |    %4d |  %4d |  %s |\n", nbBonsCRCs, acq_counter, k, score, df_value, oaci_value, type_frame, (int32_t) speed, (int32_t) Vr, (int32_t) angle, crc_show);
                                if( file_frames_dec != nullptr )
                                    fprintf(file_frames_dec, "| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |          |      |           |           |     |    %4d |    %4d |  %4d |  OK |\n", nbBonsCRCs, acq_counter, k, score, df_value, oaci_value, type_frame, (int32_t) speed, (int32_t) Vr, (int32_t) angle);

                                const int32_t ew_dir           = (vec_pack[5]&4) >> 2;
                                const int32_t ew_velocity      = ((vec_pack[5] & 3) << 8) | vec_pack[6];
                                const int32_t ns_dir           = (vec_pack[7]&0x80) >> 7;
                                const int32_t ns_velocity      = ((vec_pack[7] & 0x7f) << 3) | ((vec_pack[8] & 0xe0) >> 5);
                                const int32_t vert_rate_source = (vec_pack[8]&0x10) >> 4;
                                const int32_t vert_rate_sign   = (vec_pack[8]&0x8) >> 3;
                                const int32_t vert_rate        = ((vec_pack[8]&7) << 6) | ((vec_pack[9] & 0xfc) >> 2);
                                /* Compute velocity and angle from the two speed components. */
                                const int32_t velocity = sqrt( ns_velocity * ns_velocity + ew_velocity * ew_velocity);

                                ptr_avion->set_speed_horizontal(speed);
                                ptr_avion->set_speed_vertical(Vr);
                                ptr_avion->set_angle(angle);
                            }
                        }
#endif
#if 1
                        if ((type_frame >= 20) && (type_frame <= 22) )
                        {
                            int32_t altitude = 0;
                            for (int q = 0; q < 12; q++) {
                                if (q > 8)
                                altitude += vec_demod[39 + 9 + q] * pow(2, 10 - q - 1);
                                else if (q < 8)
                                altitude += vec_demod[39 + 9 + q] * pow(2, 10 - q);
                            }
                            altitude = altitude * 25 - 1000;

                            const int32_t CPR_format  = pack_bits      (vec_sync.data() + 53, 1);
                            const float f_latitude    = pack_bits_float(vec_sync.data() + 54, 17);//(float)enc_latitude  / 131072.0; // divise par 2^17
                            const float f_longitude   = pack_bits_float(vec_sync.data() + 71, 17);//(float)enc_longitude / 131072.0; // divise par 2^17
                            const float final_lat     = ComputeLatitude (f_latitude,  ref_latitude,                CPR_format);
                            const float final_lon     = ComputeLongitude(f_longitude, final_lat,    ref_longitude, CPR_format);
                            const int32_t dist        = distance(final_lat, final_lon, ref_latitude, ref_longitude);

                            if (dump_decoded_frame && (dump_resume == false))
                                printf("| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |   %6d |    %d |  %8.5f |  %8.5f | %3d |         |         |       |  %s |\n", nbBonsCRCs, acq_counter, k, score, df_value, oaci_value, type_frame, (int32_t)altitude, CPR_format, final_lon, final_lat, dist, crc_show);
                            if( file_frames_dec != nullptr )
                                fprintf(file_frames_dec, "| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |   %6d |    %d |  %8.5f |  %8.5f | %3d |         |         |       |  OK |\n", nbBonsCRCs, acq_counter, k, score, df_value, oaci_value, type_frame, (int32_t)altitude, CPR_format, final_lon, final_lat, dist);
                            ptr_avion->set_altitude (altitude);
                            ptr_avion->set_GNSS_mode(true);
                            ptr_avion->set_latitude (final_lat);
                            ptr_avion->set_longitude(final_lon);
                            ptr_avion->update_distance();
                        }
#endif
                        if ((type_frame >= 23) && (type_frame <= 27))
                        {
                            if (dump_decoded_frame && (dump_resume == false))
                                printf("| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |          |      |           |           |     |         |         |       |  %s |\n", nbBonsCRCs, acq_counter, k, score, df_value, oaci_value, type_frame, crc_show);
                            if( file_frames_dec != nullptr )
                                fprintf(file_frames_dec, "| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |          |      |           |           |     |         |         |       |  OK |\n", nbBonsCRCs, acq_counter, k, score, df_value, oaci_value, type_frame);
                        }

                        if (type_frame == 28)
                        {
                            if (dump_decoded_frame && (dump_resume == false))
                                printf("| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |          |      |           |           |     |         |         |       |  %s |\n", nbBonsCRCs, acq_counter, k, score, df_value, oaci_value, type_frame, crc_show);
                            if( file_frames_dec != nullptr )
                                fprintf(file_frames_dec, "| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |          |      |           |           |     |         |         |       |  OK |\n", nbBonsCRCs, acq_counter, k, score, df_value, oaci_value, type_frame);
                        }

                        if (type_frame == 29)
                        {
                            if (dump_decoded_frame && (dump_resume == false))
                                printf("| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |          |      |           |           |     |         |         |       |  %s |\n", nbBonsCRCs, acq_counter, k, score, df_value, oaci_value, type_frame, crc_show);
                            if( file_frames_dec != nullptr )
                                fprintf(file_frames_dec, "| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |          |      |           |           |     |         |         |       |  OK |\n", nbBonsCRCs, acq_counter, k, score, df_value, oaci_value, type_frame);
                        }

                        if (type_frame == 31)
                        {
                            if (dump_decoded_frame && (dump_resume == false))
                                printf("| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |          |      |           |           |     |         |         |       |  %s |\n", nbBonsCRCs, acq_counter, k, score, df_value, oaci_value, type_frame, crc_show);
                            if( file_frames_dec != nullptr )
                                fprintf(file_frames_dec, "| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |          |      |           |           |     |         |         |       |  OK |\n", nbBonsCRCs, acq_counter, k, score, df_value, oaci_value, type_frame);
                        }

                        if( (dump_decoded_frame == false) && (dump_resume == false) )
                        {
                            ptr_avion->print();
                        }
#if 0
                        for(int i = 0; i < vec_demod.size(); i += 1)
                        {
                            if(i ==  8) printf(" ");
                            else if(i == 13) printf(" ");
                            else if(i == 16) printf(" ");
                            else if(i == 40) printf(" ");
                            else if(i == 96) printf(" ");
                            printf("%d", vec_demod[i]);
                        }
                        printf("\n");
#endif
                    } else {
                        for (int i = 0; i < vec_demod.size(); i += 1) {
                            if (i == 8) printf(" ");
                            else if (i == 13) printf(" ");
                            else if (i == 16) printf(" ");
                            else if (i == 40) printf(" ");
                            else if (i == 96) printf(" ");
                            printf("%d", vec_demod[i]);
                        }
                        printf(" ");

                        int oaci = 0;
                        for (int q = 0; q < 24; q++) {
                            oaci += vec_demod[16 + q] * pow(2, 23 - q);
                        }
                        printf(" [OACI : %06X] ", oaci);

                        red();
                        printf(" [CRC KO]\n");
                        black();
                    }

                }
                timer.stop_decoding();
                k += 1; // On pourrait augementer de plus car il y a peu de chance qu'une autre
                        // trame soit présente d'ici 480 ech.
            }
        }

        acq_counter += 1;

        //
        // Tous les x secondes on affiche la liste des avions qui ont été actifs durant les y dernieres
        // secondes dans le terminal, cela permet d'avoir une vue d'ensemble des trames actuellement recues...
        //

        //////////////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        if ( (acq_counter % acq_per_sec) == 0 ) // début du stockage temps reel des données
        {
            dump_relatime.update( liste_v );
#if 0
            const int nPlanes = liste_v.size();
            FILE* rt = fopen( "/tmp/avions.gps", "w" );
            if( rt == NULL )
            {
                printf("(EE) An error occurs when openning the /tmp/avions.gps file !\n");
                exit( EXIT_FAILURE );
            }

            for (int p = 0; p < nPlanes; p += 1)
            {
                if( (liste_v.at(p)->last_update() > 600) || (liste_v.at(p)->get_messages() == 1) )
                    continue;

                const int nLon = liste_v.at(p)->list_long.size();
                const int nLat = liste_v.at(p)->list_lat.size();

                for (int t = 0; t < nLon; t += 1)
                {
                    const float lon = liste_v.at(p)->list_long.at(t);
                    const float lat = liste_v.at(p)->list_lat.at(t);
                    fprintf(rt, "%+24.22f %+24.22f %d\n", lat, lon, p);
                }
            }
            fclose( rt );
#endif
        } // fin du stockage temps reel des données
        //
        //
        //////////////////////////////////////////////////////////////////////////////////////////////////


        if (
                (dump_resume == true) && ( (acq_counter % acq_per_sec == 0)
             || (!radio->alive())
             || (isFinished == true))
           )
        {
            printf("\e[1;1H\e[2J");
            const auto stop = std::chrono::system_clock::now();
            const int32_t eTime = chrono::duration_cast<chrono::seconds>(stop - start).count();
            std::cout << "Application runtime : " << eTime << " seconds"
                      << std::endl;
            std::cout << "Number of frames over the detection value      : " << nbTramesDetectees << std::endl;
            std::cout << "Number of frames with validated CRC value      : " << nbBonsCRCs        << std::endl;
            std::cout << " - Number of initially correct  CRC values     : " << nbBonsCRC_init    << std::endl;
            std::cout << " - Number of saved frames with bit-flip (x1)   : " << nbBonsCRCs_1x     << std::endl;
            std::cout << " - Number of saved frames with bit-flip (x2)   : " << nbBonsCRCs_2x     << std::endl;
            std::cout << " - Number of saved frames with bit-flip (x3)   : " << nbBonsCRCs_3x     << std::endl;
            std::cout << "Number of discarded frames with DF = 18        : " << nbDF18Frames      << " (type == 18)" <<std::endl;
            std::cout << "Number of discarded frames with strange values : " << nbStrangeFrames   << " (type != 17 && type != 18)" <<std::endl;

            std::cout << std::endl;
            printf("OACI   | TYPE   | NAME     | CORR. SCORE       |  LATITUDE | LONGITUDE | H.SPEED   | V.SPEED   | ANGLE | TYPE | ALTITUDE  | DISTANCE [min,max] | FRAMES | LAST     |\n");
            printf("-------+--------+----------+-------------------+-----------+-----------+-----------+-----------+-------+------+-----------+--------------------+--------+----------|\n");

            for (int i = 0; i < liste_v.size(); i += 1)
            {
                if( liste_v.at(i)->get_messages() > 1 )         // Pour filter les bétises...
                    if( liste_v.at(i)->last_update() < 600 )    // On n'affiche que les avions des 10 dernieres minutes
                        liste_v.at(i)->print();
            }
        }


    }

    //
    // https://mobisoftinfotech.com/tools/plot-multiple-points-on-map/
    //
    if( file_coords != nullptr )
    {
        //
        // LATTITUDE
        //
        fprintf(file_coords, "lat = [\n");
        for (uint32_t ii = 0; ii < liste_v.size(); ii += 1)    // pour tous les avions
        {
            for (uint32_t jj = 0; jj < liste_v.at(ii)->list_lat.size(); jj += 1)    // pour tous les positions
                {
                if( jj != (liste_v.at(ii)->list_lat.size() - 1) )
                    fprintf(file_coords, "%24.22f,\n", liste_v.at(ii)->list_lat[jj]);
                else
                    fprintf(file_coords, "%24.22f\n", liste_v.at(ii)->list_lat[jj]);
                }
        }
        fprintf(file_coords, "];\n");
        fprintf(file_coords, "\n");

        //
        // LONGITUDE
        //
        fprintf(file_coords, "lon = [\n");
        for (uint32_t ii = 0; ii < liste_v.size(); ii += 1)    // pour tous les avions
        {
            for (uint32_t jj = 0; jj < liste_v.at(ii)->list_long.size(); jj += 1)    // pour tous les positions
            {
                if( jj != (liste_v.at(ii)->list_long.size() - 1) )
                    fprintf(file_coords, "%24.22f,\n", liste_v.at(ii)->list_long[jj]);
                else
                    fprintf(file_coords, "%24.22f\n", liste_v.at(ii)->list_long[jj]);
            }
        };
        fprintf(file_coords, "];\n");
        fprintf(file_coords, "\n");
        //
        // OACI = COULEUR
        //
        fprintf(file_coords, "col = [\n");
        for (uint32_t ii = 0; ii < liste_v.size(); ii += 1)    // pour tous les avions
        {
            for (uint32_t jj = 0; jj < liste_v.at(ii)->list_long.size(); jj += 1)    // pour tous les positions
            {
                if( jj != (liste_v.at(ii)->list_long.size() - 1) )
                    fprintf(file_coords, "'#%6.6X',\n", liste_v.at(ii)->get_OACI());
                else
                    fprintf(file_coords, "'#%6.6X'\n", liste_v.at(ii)->get_OACI());
            }
        }
        fprintf(file_coords, "];\n");
        fprintf(file_coords, "\n");
        fprintf(file_coords, "geoplot(lat,lon,'x');\n");
        fprintf(file_coords, "geobasemap streets\n");
    }

    //
    // On stocke les informations relatives à tous les avions que vous avons détéctés
    //

    if( file_planes != nullptr )
    {
        for (uint32_t ii = 0; ii < liste_v.size(); ii += 1)    // pour tous les avions
        {
            if( liste_v.at(ii)->get_messages() > 1 )            // Pour filter les bétises...
                liste_v.at(ii)->store(file_planes);
        }
    }

    //
    // Affichage des statistiques d'execution
    //

    printf("\n================================================================\n");
    std::cout << "loading    : " << timer.loading()    << std::endl;
    std::cout << "conversion : " << timer.conversion() << std::endl;
    std::cout << "detection  : " << timer.detection()  << std::endl;
    std::cout << "decoding   : " << timer.decoding()   << std::endl;
    printf("================================================================\n");

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    double RTConstr = 1000.0f / (param.toDouble("fe") / buffer.size());
    double avgTime = chrono::duration_cast<chrono::milliseconds>(elapsed).count() / (float) acq_counter;

    printf("\n");
    std::cout << "Nombre d'aquisition réalisées  : " << acq_counter << std::endl;
    std::cout << "Temps moyen par acquisition    : " << avgTime << " ms" << std::endl;
    std::cout << "Constrainte tps réel / iter    : " << RTConstr << " ms" << std::endl;

    std::cout << "Number of frames over the detection value      : " << nbTramesDetectees << std::endl;
    std::cout << "Number of frames with validated CRC value      : " << nbBonsCRCs        << std::endl;
    std::cout << " - Number of initially correct  CRC values     : " << nbBonsCRC_init    << std::endl;
    std::cout << " - Number of saved frames with bit-flip (x1)   : " << nbBonsCRCs_1x     << std::endl;
    std::cout << " - Number of saved frames with bit-flip (x2)   : " << nbBonsCRCs_2x     << std::endl;
    std::cout << " - Number of saved frames with bit-flip (x3)   : " << nbBonsCRCs_3x     << std::endl;
    std::cout << "Number of discarded frames with strange values : " << nbStrangeFrames   << " (type != 17)" <<std::endl;

    printf("\n");
    std::cout << "Nombre de trames emises  (frames)  = " << nbBonsCRCs << std::endl;

    printf("\n");
    std::cout << "Temps total : " << chrono::duration_cast<chrono::milliseconds>(end - start).count() << " ms" << std::endl;
    printf("\n");

    //
    // Si on a loggué les données dans des fichiers, il est temps de les fermer proprement...
    //

    if( StoreDataSet == true )
    {
        //fclose(file_iq_raw    );
        //fclose(file_frames_bin);
        fclose(file_frames_dec);
        fclose(file_planes    );
        fclose(file_coords    );
    }

    //
    // On fait le ménage parmis les objects que nous avons crée puis on quitte
    //

    delete radio;
    delete detect;
    delete conv;

    return 0;
}

#pragma clang diagnostic pop