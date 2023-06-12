#pragma clang diagnostic push
#pragma ide diagnostic ignored "bugprone-reserved-identifier"
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
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
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
#include "Tools/Avion/type_aircraft.hpp"
#include "Tools/Parameters/Parameters.hpp"
#include "Tools/Statistiques/Statistiques.hpp"
#include "Radio/Receiver/Library/ReceiverLibrary.hpp"
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
#include "Export/ExportRealTime.hpp"
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
#include "Processing/CplxModule/Library/CplxModuleLibrary.hpp"
#include "Processing/Detector/Library/DetectorLibrary.hpp"
#include "Processing/ADSBSynchro/RemoveADSBSynchro.hpp"
#include "Processing/DataPacking/BitPacking.hpp"
#include "Processing/CRC32b/CheckCRC32b/CheckCRC32b.hpp"
#include "Processing/CRC32b/RemoveCRC32b/RemoveCRC32b.hpp"
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
#include "Processing/Sampling/DownSampling.hpp"
#include "Processing/PPM/PPM_demod.hpp"
#include "Tools/colors.hpp"
#include "tools.hpp"
#include "Tools/Avion/Avion.hpp"
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
using namespace std;
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
bool isFinished = false;
void my_ctrl_c_handler(int s)
{
    if (isFinished == true) {
        exit(EXIT_FAILURE);
    }
    isFinished = true;
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
struct LLR
{
    int index;
    float    value;
};

bool compareLLRs(const LLR a, const LLR b)
{
    return a.value < b.value;
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
int decode_ac12_field(const unsigned char *msg)
{
    int q_bit = msg[5] & 1;
    if (q_bit) {
        // N is the 11 bit integer resulting from the removal of bit Q
        //*unit = MODE_S_UNIT_FEET;
        int n = ((msg[5] >> 1) << 4) | ((msg[6] & 0xF0) >> 4);
        // The final altitude is due to the resulting number multiplied by 25,
        // minus 1000.
        return n * 25 - 1000;
    } else {
        return 0;
    }
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
void dump( std::vector<uint8_t>& vec_sync )
{
    const int length = vec_sync.size();
    printf("  [");
    for(int i = 0; i < length; i += 1)
    {
        if( i == 33 ) printf(" ");
        if( i == 38 ) printf(" ");
        if( i == 40 ) printf(" ");
        if( i == 41 ) printf(" ");
        if( i == 53 ) printf(" ");
        if( i == 54 ) printf(" ");
        if( i == 55 ) printf(" ");
        if( i == 72 ) printf(" ");
        if( i == 89 ) printf(" ");
        printf("%d", vec_sync[i] );

    }
    printf("];\n");
}

//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
// Decode the 13 bit AC altitude field (in DF 20 and others). Returns the
// altitude, and set 'unit' to either MODE_S_UNIT_METERS or MDOES_UNIT_FEETS.
int decode_ac13_field(const unsigned char *msg)
{
    int m_bit = msg[3] & (1<<6);
    int q_bit = msg[3] & (1<<4);
    if (!m_bit)
    {
        if (q_bit) {
            // N is the 11 bit integer resulting from the removal of bit Q and M
            int n = ((msg[2]&31)<<6) |
                    ((msg[3]&0x80)>>2) |
                    ((msg[3]&0x20)>>1) |
                    (msg[3]&15);
            // The final altitude is due to the resulting number multiplied by
            // 25, minus 1000.
            return n*25-1000;
        } else {
            // TODO: Implement altitude where Q=0 and M=0
        }
    } else {
        //*unit = MODE_S_UNIT_METERS;
        // TODO: Implement altitude when meter unit is selected.
        return -1;
    }
    return 0;
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
int extract_odd_even(const uint8_t* vec_pack)
{
    return ((vec_pack[6] >> 2) & 0x01);
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
uint32_t extract_latitude(const uint8_t* vec_pack)
{
    uint32_t r;
    r  = ( ((uint32_t)vec_pack[6] & 3) << 15);
    r |= ( ((uint32_t)vec_pack[7]    ) <<  7);
    r |= ( ((uint32_t)vec_pack[8]    ) >>  1);
    return r;
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
uint32_t extract_longitude(const uint8_t* vec_pack)
{
    uint32_t r;
    r  = ( ((uint32_t)vec_pack[ 8] & 1) << 16);
    r |= ( ((uint32_t)vec_pack[ 9]    ) <<  8);
    r |=   ((uint32_t)vec_pack[10]           );
    return r;
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
int check_extract_odd_even(const uint8_t* vec_pack, const uint8_t* vec_sync)
{
    const int odd_even   = extract_odd_even (vec_pack         );
    const int CPR_format = pack_bits        (vec_sync + 53,  1);
    if( CPR_format != odd_even )
    {
        printf("(EE) T18 CPR_format != odd_even (%d / %d)\n", CPR_format, odd_even);
    }
    return odd_even;
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
uint32_t check_extract_longitude(const uint8_t* vec_pack, const uint8_t* vec_sync)
{
    const int raw_longitude   = extract_longitude(vec_pack);
    const int raw_longitude_2 = pack_bits(vec_sync + 71, 17);
    if( raw_longitude != raw_longitude_2 )
    {
        printf("(EE) T18 raw_latitude != raw_latitude_2 (%d / %d)\n", raw_longitude, raw_longitude_2);
    }
    return raw_longitude;
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
uint32_t check_extract_latitude(const uint8_t* vec_pack, const uint8_t* vec_sync)
{
    const int raw_latitude    = extract_latitude (vec_pack         );
    const int raw_latitude_2  = pack_bits        (vec_sync + 54, 17);
    if( raw_latitude != raw_latitude_2 )
    {
        printf("(EE) T18 raw_latitude != raw_latitude_2 (%d / %d)\n", raw_latitude, raw_latitude_2);
    }
    return raw_latitude;
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
#include "brute_force/brute_force_1x.hpp"
#include "brute_force/brute_force_2x.hpp"
#include "brute_force/brute_force_3x.hpp"
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
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
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
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


/* This algorithm comes from:
 * http://www.lll.lu/~edward/edward/adsb/DecodingADSBposition.html.
 *
 *
 * A few remarks:
 * 1) 131072 is 2^17 since CPR latitude and longitude are encoded in 17 bits.
 * 2) We assume that we always received the odd packet as last packet for
 *    simplicity. This may provide a position that is less fresh of a few
 *    seconds.
 */
/* Always positive MOD operation, used for CPR decoding. */
int cprModFunction(int a, int b) {
    int res = a % b;
    if (res < 0) res += b;
    return res;
}
/* The NL function uses the precomputed table from 1090-WP-9-14 */
int cprNLFunction(double lat) {
    if (lat < 0) lat = -lat; /* Table is simmetric about the equator. */
    if (lat < 10.47047130) return 59;
    if (lat < 14.82817437) return 58;
    if (lat < 18.18626357) return 57;
    if (lat < 21.02939493) return 56;
    if (lat < 23.54504487) return 55;
    if (lat < 25.82924707) return 54;
    if (lat < 27.93898710) return 53;
    if (lat < 29.91135686) return 52;
    if (lat < 31.77209708) return 51;
    if (lat < 33.53993436) return 50;
    if (lat < 35.22899598) return 49;
    if (lat < 36.85025108) return 48;
    if (lat < 38.41241892) return 47;
    if (lat < 39.92256684) return 46;
    if (lat < 41.38651832) return 45;
    if (lat < 42.80914012) return 44;
    if (lat < 44.19454951) return 43;
    if (lat < 45.54626723) return 42;
    if (lat < 46.86733252) return 41;
    if (lat < 48.16039128) return 40;
    if (lat < 49.42776439) return 39;
    if (lat < 50.67150166) return 38;
    if (lat < 51.89342469) return 37;
    if (lat < 53.09516153) return 36;
    if (lat < 54.27817472) return 35;
    if (lat < 55.44378444) return 34;
    if (lat < 56.59318756) return 33;
    if (lat < 57.72747354) return 32;
    if (lat < 58.84763776) return 31;
    if (lat < 59.95459277) return 30;
    if (lat < 61.04917774) return 29;
    if (lat < 62.13216659) return 28;
    if (lat < 63.20427479) return 27;
    if (lat < 64.26616523) return 26;
    if (lat < 65.31845310) return 25;
    if (lat < 66.36171008) return 24;
    if (lat < 67.39646774) return 23;
    if (lat < 68.42322022) return 22;
    if (lat < 69.44242631) return 21;
    if (lat < 70.45451075) return 20;
    if (lat < 71.45986473) return 19;
    if (lat < 72.45884545) return 18;
    if (lat < 73.45177442) return 17;
    if (lat < 74.43893416) return 16;
    if (lat < 75.42056257) return 15;
    if (lat < 76.39684391) return 14;
    if (lat < 77.36789461) return 13;
    if (lat < 78.33374083) return 12;
    if (lat < 79.29428225) return 11;
    if (lat < 80.24923213) return 10;
    if (lat < 81.19801349) return 9;
    if (lat < 82.13956981) return 8;
    if (lat < 83.07199445) return 7;
    if (lat < 83.99173563) return 6;
    if (lat < 84.89166191) return 5;
    if (lat < 85.75541621) return 4;
    if (lat < 86.53536998) return 3;
    if (lat < 87.00000000) return 2;
    else return 1;
}

int cprNFunction(double lat, int isodd) {
    int nl = cprNLFunction(lat) - isodd;
    if (nl < 1) nl = 1;
    return nl;
}

double cprDlonFunction(double lat, int isodd) {
    return 360.0 / cprNFunction(lat, isodd);
}

void decodeCPR(
        int even_cprlat,
        int odd_cprlat,
        int even_cprlon,
        int odd_cprlon,
        int even,
        double &lon,
        double &lat
)
{
    const double AirDlat0 = 360.0 / 60;
    const double AirDlat1 = 360.0 / 59;

    double lat0 = even_cprlat;
    double lat1 = odd_cprlat;
    double lon0 = even_cprlon;
    double lon1 = odd_cprlon;

    /* Compute the Latitude Index "j" */
    int j = floor(((59*lat0 - 60*lat1) / 131072) + 0.5);
    double rlat0 = AirDlat0 * (cprModFunction(j,60) + lat0 / 131072);
    double rlat1 = AirDlat1 * (cprModFunction(j,59) + lat1 / 131072);

    if (rlat0 >= 270) rlat0 -= 360;
    if (rlat1 >= 270) rlat1 -= 360;

    /* Check that both are in the same latitude zone, or abort. */
    if (cprNLFunction(rlat0) != cprNLFunction(rlat1)) return;

    /* Compute ni and the longitude index m */
    if ( even ){
        /* Use even packet. */
        int ni = cprNFunction(rlat0,0);
        int m = floor((((lon0 * (cprNLFunction(rlat0)-1)) -
                        (lon1 * cprNLFunction(rlat0))) / 131072) + 0.5);
        lon = cprDlonFunction(rlat0,0) * (cprModFunction(m,ni)+lon0/131072);
        lat = rlat0;
    } else {
        /* Use odd packet. */
        int ni = cprNFunction(rlat1,1);
        int m = floor((((lon0 * (cprNLFunction(rlat1)-1)) -
                        (lon1 * cprNLFunction(rlat1))) / 131072.0) + 0.5);
        lon = cprDlonFunction(rlat1,1) * (cprModFunction(m,ni)+lon1/131072);
        lat = rlat1;
    }
    if (lon > 180) lon -= 360;
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
int main(int argc, char *argv[])
{
    printf("(II) ==================================== ADSB ====================================\n");
    printf("(II)  par Florian LOUPIAS - Février 2020\n"         );
    printf("(II)  par Bertrand LE GAL - Octobre 2020 -> .....\n");
    printf("(II) ==================================== ADSB ====================================\n");

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
    param.set("mode_conv", "NEON");       // scalar
    param.set("mode_corr", "NEON_Inter"); // scalar
#elif defined(__AVX2__)
    param.set("mode_conv", "AVX2"); // scalar
    param.set("mode_corr", "AVX2_Inter"); // scalar
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
                    {"verbose",         no_argument,       NULL, 'v'},  // affiche temps sur chaque boucle Np + cplx => abs
                    {"seuil",           required_argument, NULL, 's'},  // pour changer la valeur min de la correlation (synchro)
                    {"np",              required_argument, NULL, 'n'},  // pour changer le nombre de boucle Np (ie nbre echantillon*200000) // Np = 10 => 0.5 s
                    {"conv",            required_argument, NULL, 'c'}, // a partir d'un fichier
                    {"corr",            required_argument, NULL, 'd'}, // a partir d'un fichier
                    {"radio",           required_argument, NULL, 'r'}, // a partir d'un fichier
                    {"file",            required_argument, NULL, 'F'}, // a partir d'un fichier
                    {"file-stream",     required_argument, NULL, 'Q'}, // a partir d'un fichier
                    {"fc",              required_argument, NULL, 'f'}, // changer la frequence de la porteuse
                    {"fe",              required_argument, NULL, 'e'}, // changer la frequence echantillonnage
                    {"payload",         required_argument, NULL, 'p'}, // changer la frequence echantillonnage
                    {"amplifier",       required_argument, NULL, 'A'}, // changer la frequence echantillonnage
                    {"rcv_gain",        required_argument, NULL, 'L'}, // changer la frequence echantillonnage
                    {"rcv-gain",        required_argument, NULL, 'L'}, // changer la frequence echantillonnage
                    {"brute-force-1x",  no_argument,       NULL, '1'}, // changer la frequence echantillonnage
                    {"brute-force-2x",  no_argument,       NULL, '2'}, // changer la frequence echantillonnage
                    {"brute-force-3x",  no_argument,       NULL, '3'}, // changer la frequence echantillonnage
                    {"brute-force-llr", no_argument,       NULL, 'X'}, // changer la frequence echantillonnage
                    {"inter",           no_argument,       NULL, 'I'}, // changer la frequence echantillonnage
                    {"intra",           no_argument,       NULL, 'i'}, // changer la frequence echantillonnage
                    {"dump-frames",     no_argument,       NULL, 'D'}, // changer la frequence echantillonnage
                    {"ppm",             required_argument, NULL, 'P'}, // changer la frequence echantillonnage
                    {"liste",           no_argument,       NULL, 'l'}, // changer la frequence echantillonnage
                    {"buffer_size",     required_argument, NULL, 'B'}, // changer la frequence echantillonnage
                    {"ENSEIRB",         no_argument,       NULL, 'E'}, // changer la frequence echantillonnage
                    {NULL, 0,                              NULL, 0}
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

    Statistiques stats;

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

    const float ps_min         = param.toFloat("ps_min");
    const bool brute_force_1x  = param.toBool ("brute_force_1x");
    const bool brute_force_2x  = param.toBool ("brute_force_2x");
    const bool brute_force_3x  = param.toBool ("brute_force_3x");
//    const bool brute_force_llr = param.toBool ("brute_force_llr");

    std::vector<float> buffer_abs;
    std::vector<float> buffer_detect;

    DownSampling down(2);
    PPM_demod ppd;

    std::vector<uint8_t> buff_6;
    std::vector<uint8_t> buff_7;

    const int overSampling    =  2;
    const int payload_nBytes  = 14;
    const int payload_nBits   =  8 * payload_nBytes;             // 112
    const int frame_nBits     =  8 + payload_nBits;              // 120
    const int modulation_nIQs =  2 * frame_nBits;                // 240
    const int oversample_nIQs = overSampling * modulation_nIQs; // 480

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

    FILE* file_frames_dec = nullptr; // Le fichier contenant les trames décodées
    FILE* file_planes     = nullptr; // Le fichier contenant les informations concernant les avions
    FILE* file_coords     = nullptr; // Le fichier contenant les informations concernant les avions

    if( StoreDataSet == true )
    {
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
    int n_dots      = 0;

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
        bool error_n = radio->reception(buffer, coverage);

        if (error_n == false) // Cela signifie que l'on a rencontré une erreur lors de la
            continue;         // reception des echantillons

        //
        // CALCUL DU MODULE DES VOIES (I,Q)
        //
        conv->execute(&buffer, &buffer_abs);

        //
        // ON LANCE LA FONCTION DE DETECTION SUR L'ENSEMBLE DU BUFFER
        //
        detect->execute(&buffer_abs, &buffer_detect);


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

                stats.add_detected_frame(); // on met a jour les stats
//              nbTramesDetectees += 1;    // On vient de detecter qqchose

                for (int x = 0; x < vec_data.size(); x += 1)    // On extrait les 120 bits (x2) du vecteur
                    vec_data[x] = buffer_abs[x + k];            // d'echantillons (modules complexes de IQ)

                o_down.execute(vec_data,  vec_down );    // 480 values => 240 values
                o_ppm.execute (vec_down,  vec_demod);       // 240 values => 120 bits
                o_sync.execute(vec_demod, vec_sync );        // 112 bits
                o_pack.execute(vec_sync,  vec_pack );     // 15 octets

                bool crc_is_ok = check_crc  <112 / 8>( vec_pack.data() );
                if( crc_is_ok )
                    stats.validated_crc_init();

                //
                // On stocke les données pour une utilisation ultérieure
                //

                bool crc_brute_1x = false;
                if ( (crc_is_ok == false) && (brute_force_1x == true) )
                {
                    crc_brute_1x = try_brute_force_1x( vec_pack, vec_demod, vec_sync );
                    if( crc_brute_1x ) stats.validated_crc_brute_1x();
                }
                crc_is_ok |= crc_brute_1x;

                bool crc_brute_2x = false;
#if 1
                if ( (crc_is_ok == false) && (brute_force_2x == true) )
                {
                    crc_brute_2x = try_brute_force_2x( vec_pack, vec_demod, vec_sync );
                    if( crc_brute_2x ) stats.validated_crc_brute_2x();
                    printf("On est passe ici (%s %d)\n", __FILE__, __LINE__);
                    exit( EXIT_FAILURE );
                }
#endif
                crc_is_ok |= crc_brute_2x;

                bool crc_brute_3x = false;
#if 1
                if ( (crc_is_ok == false) && (brute_force_3x == true) )
                {
                    crc_brute_3x = try_brute_force_3x( vec_pack, vec_demod, vec_sync );
                    if( crc_brute_3x ) stats.validated_crc_brute_3x();
                    printf("On est passe ici (%s %d)\n", __FILE__, __LINE__);
                    exit( EXIT_FAILURE );
                }
#endif
                crc_is_ok |= crc_brute_3x;

//                bool crc_brute_llr = false;
//                if ( (crc_initial == false) && (crc_brute_1x == false) && (crc_brute_2x == false) && (brute_force_llr == true)  )
//                {
//                    crc_brute_llr = try_brute_force_llrs( vec_pack, vec_demod, vec_sync, vec_down );
//                }


                if ((crc_is_ok == true) || (crc_is_ok == false && verbose >= 2))
                {
                    const char* crc_show;
                         if( crc_brute_1x  == true ) crc_show = "\x1B[33mOK\x1B[0m";    //
                    else if( crc_brute_2x  == true ) crc_show = "\x1B[31;1mOK\x1B[0m";  //
                    else if( crc_brute_3x  == true ) crc_show = "\x1B[31mOK\x1B[0m";    //
                    else                             crc_show = "\x1B[32mOK\x1B[0m";    // CRC OK d'origine

                    if( verbose >= 1 )
                    {
                        printf("  [");
                        for(int32_t i = 0; i < (int)vec_sync.size(); i += 1)
                            printf("%d ", vec_sync[i] );
                        printf("];\n");
                    }

                    if (crc_is_ok)
                    {
                        const int32_t df_value   = pack_bits(vec_sync.data(), 5);
                        const int32_t type_frame = pack_bits(vec_sync.data() + 32, 5);
                        const int32_t oaci_value = pack_bits(vec_sync.data() + 8, 24);

                        if( df_value == 18 ){
                            stats.add_type_18_frame();//nbDF18Frames += 1;
                            continue;
                        }else if( df_value != 17 ){
                            stats.add_strange_frame();
                            //nbStrangeFrames += 1;
                            continue;
                        }

                        if( file_frames_dec != nullptr )
                        {
                            if( (crc_brute_1x || crc_brute_2x || crc_brute_3x )   == false )
                                fprintf(file_frames_dec, "+ ");
                            else
                                fprintf(file_frames_dec, "- ");
                            for(int o = 0; o < 14; o += 1)
                                fprintf(file_frames_dec, "%2.2X", vec_pack[o]);
                            fprintf(file_frames_dec, " | ");
                        }

                        //
                        //
                        //
                        ////////////////////////////////////////////////////////////////////
                        //
                        //
                        //
                        Avion *ptr_avion = liste_m[oaci_value];
                        if (ptr_avion == nullptr) {
                            ptr_avion = new Avion(oaci_value);
                            liste_m[oaci_value] = ptr_avion;
                            liste_v.push_back(ptr_avion);
                        }
                        ptr_avion->update();                    // on indique qu'on vient de recevoir une trame pour l'avion
                        ptr_avion->set_score( score );    // on memorise le score obtenu lors de la correlation
                        //
                        //
                        //
                        ////////////////////////////////////////////////////////////////////
                        //
                        //
                        //
                        if ((type_frame >= 1) && (type_frame <= 4))
                        {
                            const int32_t type_airc    = pack_bits(vec_sync.data() + 37, 3);
                            const int32_t typeAricraft = toCode( type_frame, type_airc );
                            if( typeAricraft == -1 )
                            {
                                stats.add_strange_frame();
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
                                printf("| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d | %s |          |      |           |           |     |         |         |       |  %s |\n", stats.validated_crc(), acq_counter, k, score, df_value, oaci_value, type_frame, caractere, crc_show);
                            if( file_frames_dec != nullptr )
                                fprintf(file_frames_dec, "| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d | %s |          |      |           |           |     |         |         |       |  OK |\n", stats.validated_crc(), acq_counter, k, score, df_value, oaci_value, type_frame, caractere);

                            ptr_avion->set_type(typeAricraft);
                            ptr_avion->set_name(caractere   );
                        }
                        //
                        //
                        //
                        ////////////////////////////////////////////////////////////////////
                        //
                        //
                        //
                        if ((type_frame >= 5) && (type_frame <= 8) )
                        {
                            if (dump_decoded_frame && (dump_resume == false))
                                printf("| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |          |      |           |           |     |         |         |       |  %s |\n", stats.validated_crc(), acq_counter, k, score, df_value, oaci_value, type_frame, crc_show);
                            if( file_frames_dec != nullptr )
                                fprintf(file_frames_dec, "| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |          |      |           |           |     |         |         |       |  OK |\n", stats.validated_crc(), acq_counter, k, score, df_value, oaci_value, type_frame);
                        }
                        //
                        //
                        //
                        ////////////////////////////////////////////////////////////////////
                        //
                        //
                        //
                        if( (type_frame >= 9) && (type_frame <= 18) )
                        {
                            //
                            const int altitude = decode_ac12_field( vec_pack.data() );

                            const int odd_even        = check_extract_odd_even ( vec_pack.data(), vec_sync.data() );
                            const int raw_latitude    = check_extract_latitude ( vec_pack.data(), vec_sync.data() );
                            const int raw_longitude   = check_extract_longitude( vec_pack.data(), vec_sync.data() );

                            int last_lon, last_lat;
                            double lon, lat;
                            if( odd_even == 0 ) // EVEN frame
                            {
                                last_lon = ptr_avion->lon_even;
                                last_lat = ptr_avion->lat_even;
                                ptr_avion->lat_even = raw_latitude;
                                ptr_avion->lon_even = raw_longitude;
                                if( 1 ) //(ptr_avion->lat_odd == 0) || (ptr_avion->lon_odd == 0) )
                                {
                                    const float f_latitude   = pack_bits_float(vec_sync.data() + 54, 17);
                                    const float f_longitude  = pack_bits_float(vec_sync.data() + 71, 17);
                                    lat    = ComputeLatitude (f_latitude,  ref_latitude,                 odd_even);
                                    lon    = ComputeLongitude(f_longitude, lat,           ref_longitude, odd_even);
                                }
                                else
                                {
                                    decodeCPR(ptr_avion->lat_even, ptr_avion->lat_odd, ptr_avion->lon_even, ptr_avion->lon_odd, 1, lon, lat);
                                }
                            }
                            else  // ODD frame
                            {
                                last_lon = ptr_avion->lon_odd;
                                last_lat = ptr_avion->lat_odd;
                                ptr_avion->lat_odd = raw_latitude;
                                ptr_avion->lon_odd = raw_longitude;
                                if( 1 ) // (ptr_avion->lat_even == 0) || (ptr_avion->lon_even == 0) )
                                {
                                    const float f_latitude   = pack_bits_float(vec_sync.data() + 54, 17);
                                    const float f_longitude  = pack_bits_float(vec_sync.data() + 71, 17);
                                    lat    = ComputeLatitude (f_latitude,  ref_latitude,                 odd_even);
                                    lon    = ComputeLongitude(f_longitude, lat,           ref_longitude, odd_even);
                                }
                                else
                                {
                                    decodeCPR(ptr_avion->lat_even, ptr_avion->lat_odd, ptr_avion->lon_even, ptr_avion->lon_odd, 0, lon, lat);
                                }
                            }
                            // END NEW BLG

                            const int32_t dist = distance(lat, lon, ref_latitude, ref_longitude);

                            if (dump_decoded_frame && (dump_resume == false)) {
                                printf("| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |   %6d |    %d |  %8.5f |  %8.5f | %3d |         |         |       |  %s |\n", stats.validated_crc(), acq_counter, k, score, df_value, oaci_value, type_frame, (int32_t)altitude, odd_even, (float)lon, (float)lat, dist, crc_show);
                            }
                            if( file_frames_dec != nullptr )
                                fprintf(file_frames_dec, "| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |   %6d |    %d |  %8.5f |  %8.5f | %3d |         |         |       |  OK |\n", stats.validated_crc(), acq_counter, k, score, df_value, oaci_value, type_frame, (int32_t)altitude, odd_even, (float)lon, (float)lat, dist);

#if 0
                            if( ptr_avion->get_latitude() != 0.0f )
                            {
                                float diff_x = abs( ptr_avion->get_latitude()  - lat );
                                float diff_y = abs( ptr_avion->get_longitude() - lon );
                                if( (diff_x > 0.5) || (diff_y > 0.5) )
                                {
//                                  isFinished = true;
                                    bool retest = check_crc  <112 / 8>( vec_pack.data() );

                                    printf("crc_is_ok value     = %d\n", crc_is_ok);
                                    printf("Frame CRC value     = %d\n", retest);
                                    printf("CPR_format          = %d\n", CPR_format);
                                    printf("last_lat            = %d\n", last_lat);
                                    printf("last_lon            = %d\n", last_lon);
                                    printf("ptr_avion->lat_odd  = %d\n", ptr_avion->lat_odd);
                                    printf("ptr_avion->lon_odd  = %d\n", ptr_avion->lon_odd);
                                    printf("ptr_avion->lat_even = %d\n", ptr_avion->lat_even);
                                    printf("ptr_avion->lon_even = %d\n", ptr_avion->lon_even);
//                                  k = length;
                                }
                            }
#endif
#if 1
                            ptr_avion->set_altitude   (altitude);
                            ptr_avion->set_GNSS_mode  (false);
                            ptr_avion->set_latitude   (lat);
                            ptr_avion->set_longitude  (lon);
                            ptr_avion->set_reliability( !(crc_brute_1x || crc_brute_2x || crc_brute_3x) );
                            ptr_avion->update_distance();
#else

                            if( ptr_avion->get_latitude() != 0.0f )
                            {
                                const int last_dist = distance(lat, lon, ptr_avion->get_latitude(), ptr_avion->get_longitude());
                                const int maxi_dist = 40;//1.0 * ptr_avion->last_update();
                                if( last_dist < maxi_dist )
                                {
                                    ptr_avion->set_altitude   (altitude);
                                    ptr_avion->set_GNSS_mode  (false);
                                    ptr_avion->set_latitude   (lat);
                                    ptr_avion->set_longitude  (lon);
                                    ptr_avion->set_reliability( !(crc_brute_1x || crc_brute_2x || crc_brute_3x) );
                                    ptr_avion->update_distance();
                                }
                                else
                                {
                                    // la position est vraiment louche !
                                }
                            }
                            else
                            {
                                ptr_avion->set_altitude   (altitude);
                                ptr_avion->set_GNSS_mode  (false);
                                ptr_avion->set_latitude   (lat);
                                ptr_avion->set_longitude  (lon);
                                ptr_avion->set_reliability( !(crc_brute_1x || crc_brute_2x || crc_brute_3x) );                            
                                ptr_avion->update_distance();
                            }
#endif
                        }
                        //
                        //
                        //
                        ////////////////////////////////////////////////////////////////////
                        //
                        //
                        //
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
                                    printf("| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |          |      |           |           |     |    %4d |    %4d |  %4d |  %s |\n", stats.validated_crc(), acq_counter, k, score, df_value, oaci_value, type_frame, (int32_t) speed, (int32_t) Vr, (int32_t) angle, crc_show);
                                if( file_frames_dec != nullptr )
                                    fprintf(file_frames_dec, "| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |          |      |           |           |     |    %4d |    %4d |  %4d |  OK |\n", stats.validated_crc(), acq_counter, k, score, df_value, oaci_value, type_frame, (int32_t) speed, (int32_t) Vr, (int32_t) angle);

                                const int32_t ew_dir           = (vec_pack[5]&4) >> 2;
                                const int32_t ew_velocity      = ((vec_pack[5] & 3) << 8) | vec_pack[6];
                                const int32_t ns_dir           = (vec_pack[7]&0x80) >> 7;
                                const int32_t ns_velocity      = ((vec_pack[7] & 0x7f) << 3) | ((vec_pack[8] & 0xe0) >> 5);
                                const int32_t vert_rate_source = (vec_pack[8]&0x10) >> 4;
                                const int32_t vert_rate_sign   = (vec_pack[8]&0x8 ) >> 3;
                                const int32_t vert_rate        = ((vec_pack[8]&7) << 6) | ((vec_pack[9] & 0xfc) >> 2);

                                /* Compute velocity and angle from the two speed components. */

                                const int32_t velocity = sqrt( ns_velocity * ns_velocity + ew_velocity * ew_velocity);

                                ptr_avion->set_speed_horizontal(speed);
                                ptr_avion->set_speed_vertical(Vr);
                                ptr_avion->set_angle(angle);
                            }
                        }
                        //
                        //
                        //
                        ////////////////////////////////////////////////////////////////////
                        //
                        //
                        //
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

                            int nAlt = decode_ac13_field( vec_pack.data() );
                            if( altitude != nAlt )
                            {
                                printf("(EE) altitude != nAlt (%d / %d)\n", altitude, nAlt);
                                printf("(EE) altitude != nAlt (%d / %d)\n", altitude, nAlt);
                                printf("(EE) altitude != nAlt (%d / %d)\n", altitude, nAlt);
                                printf("(EE) altitude != nAlt (%d / %d)\n", altitude, nAlt);
                                printf("(EE) altitude != nAlt (%d / %d)\n", altitude, nAlt);
                                printf("(EE) altitude != nAlt (%d / %d)\n", altitude, nAlt);
                                printf("(EE) altitude != nAlt (%d / %d)\n", altitude, nAlt);
                                printf("(EE) altitude != nAlt (%d / %d)\n", altitude, nAlt);
                            }

                            // NEW BLG
////
                            const int odd_even        = check_extract_odd_even ( vec_pack.data(), vec_sync.data() );
                            const int raw_latitude    = check_extract_latitude ( vec_pack.data(), vec_sync.data() );
                            const int raw_longitude   = check_extract_longitude( vec_pack.data(), vec_sync.data() );

                            double lon, lat;
                            if( odd_even == 0 ) // EVEN frame
                            {
                                ptr_avion->lat_even = raw_latitude;
                                ptr_avion->lon_even = raw_longitude;
                                if( 1 ) // (ptr_avion->lat_odd == 0) || (ptr_avion->lon_odd == 0) )
                                {
                                    const float f_latitude   = pack_bits_float(vec_sync.data() + 54, 17);
                                    const float f_longitude  = pack_bits_float(vec_sync.data() + 71, 17);
                                    lat    = ComputeLatitude (f_latitude,  ref_latitude,                 odd_even);
                                    lon    = ComputeLongitude(f_longitude, lat,           ref_longitude, odd_even);
                                }
                                else
                                {
                                    decodeCPR(ptr_avion->lat_even, ptr_avion->lat_odd, ptr_avion->lon_even, ptr_avion->lon_odd, 1, lon, lat);
                                }
                            }
                            else  // ODD frame
                            {
                                ptr_avion->lat_odd = raw_latitude;
                                ptr_avion->lon_odd = raw_longitude;
                                if( 1 ) // (ptr_avion->lat_even == 0) || (ptr_avion->lon_even == 0) )
                                {
                                    const float f_latitude   = pack_bits_float(vec_sync.data() + 54, 17);
                                    const float f_longitude  = pack_bits_float(vec_sync.data() + 71, 17);
                                    lat    = ComputeLatitude (f_latitude,  ref_latitude,                 odd_even);
                                    lon    = ComputeLongitude(f_longitude, lat,           ref_longitude, odd_even);
                                }
                                else
                                {
                                    decodeCPR(ptr_avion->lat_even, ptr_avion->lat_odd, ptr_avion->lon_even, ptr_avion->lon_odd, 0, lon, lat);
                                }
                            }
                            // END NEW BLG

                            const int32_t dist = distance(lat, lon, ref_latitude, ref_longitude);

                            if (dump_decoded_frame && (dump_resume == false))
                                printf("| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |   %6d |    %d |  %8.5f |  %8.5f | %3d |         |         |       |  %s |\n", stats.validated_crc(), acq_counter, k, score, df_value, oaci_value, type_frame, (int32_t)altitude, odd_even, (float)lon, (float)lat, dist, crc_show);
                            if( file_frames_dec != nullptr )
                                fprintf(file_frames_dec, "| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |   %6d |    %d |  %8.5f |  %8.5f | %3d |         |         |       |  OK |\n", stats.validated_crc(), acq_counter, k, score, df_value, oaci_value, type_frame, (int32_t)altitude, odd_even, (float)lon, (float)lat, dist);

#if 1
                            ptr_avion->set_altitude   (altitude);
                            ptr_avion->set_GNSS_mode  (true);
                            ptr_avion->set_latitude   (lat);
                            ptr_avion->set_longitude  (lon);
                            ptr_avion->set_reliability( !(crc_brute_1x || crc_brute_2x || crc_brute_3x) );
                            ptr_avion->update_distance();
#else
                            if( ptr_avion->get_latitude() != 0.0f )
                            {
                                const int last_dist = distance(lat, lon, ptr_avion->get_latitude(), ptr_avion->get_longitude());
                                const int maxi_dist = 40;//1.0 * ptr_avion->last_update();
                                if( last_dist < maxi_dist )
                                {
                                    ptr_avion->set_altitude   (altitude);
                                    ptr_avion->set_GNSS_mode  (true);
                                    ptr_avion->set_latitude   (lat);
                                    ptr_avion->set_longitude  (lon);
                                    ptr_avion->set_reliability( !(crc_brute_1x || crc_brute_2x || crc_brute_3x) );
                                    ptr_avion->update_distance();
                                }
                                else
                                {
                                    // la position est vraiment louche !
                                }
                            }
                            else
                            {
                                ptr_avion->set_altitude   (altitude);
                                ptr_avion->set_GNSS_mode  (true);
                                ptr_avion->set_latitude   (lat);
                                ptr_avion->set_longitude  (lon);
                                ptr_avion->set_reliability( !(crc_brute_1x || crc_brute_2x || crc_brute_3x) );                            
                                ptr_avion->update_distance();
                            }
#endif  
                        }
                        //
                        //
                        //
                        ////////////////////////////////////////////////////////////////////
                        //
                        //
                        //
                        if ((type_frame >= 23) && (type_frame <= 27))
                        {
                            if (dump_decoded_frame && (dump_resume == false))
                                printf("| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |          |      |           |           |     |         |         |       |  %s |\n", stats.validated_crc(), acq_counter, k, score, df_value, oaci_value, type_frame, crc_show);
                            if( file_frames_dec != nullptr )
                                fprintf(file_frames_dec, "| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |          |      |           |           |     |         |         |       |  OK |\n", stats.validated_crc(), acq_counter, k, score, df_value, oaci_value, type_frame);
                        }
                        //
                        //
                        //
                        ////////////////////////////////////////////////////////////////////
                        //
                        //
                        //
                        if (type_frame == 28)
                        {
                            if (dump_decoded_frame && (dump_resume == false))
                                printf("| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |          |      |           |           |     |         |         |       |  %s |\n", stats.validated_crc(), acq_counter, k, score, df_value, oaci_value, type_frame, crc_show);
                            if( file_frames_dec != nullptr )
                                fprintf(file_frames_dec, "| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |          |      |           |           |     |         |         |       |  OK |\n", stats.validated_crc(), acq_counter, k, score, df_value, oaci_value, type_frame);
                        }
                        //
                        //
                        //
                        ////////////////////////////////////////////////////////////////////
                        //
                        //
                        //
                        if (type_frame == 29)
                        {
                            if (dump_decoded_frame && (dump_resume == false))
                                printf("| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |          |      |           |           |     |         |         |       |  %s |\n", stats.validated_crc(), acq_counter, k, score, df_value, oaci_value, type_frame, crc_show);
                            if( file_frames_dec != nullptr )
                                fprintf(file_frames_dec, "| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |          |      |           |           |     |         |         |       |  OK |\n", stats.validated_crc(), acq_counter, k, score, df_value, oaci_value, type_frame);
                        }
                        //
                        //
                        //
                        ////////////////////////////////////////////////////////////////////
                        //
                        //
                        //
                        if (type_frame == 31)
                        {
                            if (dump_decoded_frame && (dump_resume == false))
                                printf("| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |          |      |           |           |     |         |         |       |  %s |\n", stats.validated_crc(), acq_counter, k, score, df_value, oaci_value, type_frame, crc_show);
                            if( file_frames_dec != nullptr )
                                fprintf(file_frames_dec, "| %5d | %5d | %6d | %1.4f |  %2d | %06X |  %2d |          |          |      |           |           |     |         |         |       |  OK |\n", stats.validated_crc(), acq_counter, k, score, df_value, oaci_value, type_frame);
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
                        for (int i = 0; i < (int)vec_demod.size(); i += 1) {
                                 if (i ==  8) printf(" ");
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
                k += 1; // On pourrait augementer de plus car il y a peu de chance qu'une autre
                        // trame soit présente d'ici 480 ech.
            }
        }

        acq_counter += 1;


        //////////////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        // Tous les x secondes on affiche la liste des avions qui ont été actifs durant les y dernieres
        // secondes dans le terminal, cela permet d'avoir une vue d'ensemble des trames actuellement recues...
        //
        //
        if ( (acq_counter % acq_per_sec) == 0 )
        {
            dump_relatime.update( liste_v );
        }
        //
        //
        //////////////////////////////////////////////////////////////////////////////////////////////////



        //////////////////////////////////////////////////////////////////////////////////////////////////
        //
        //
        if (
                (dump_resume == true) && ( (acq_counter % acq_per_sec == 0)
             || (!radio->alive())
             || (isFinished == true))
           )
        {
            printf("\e[1;1H\e[2J");
            std::cout << std::endl;
            printf("OACI   | TYPE   | NAME     | CORR. SCORE       |  LATITUDE | LONGITUDE | H.SPEED   | V.SPEED   | ANGLE | TYPE | ALTITUDE  | DISTANCE [min,max] | FRAMES | LAST     |\n");
            printf("-------+--------+----------+-------------------+-----------+-----------+-----------+-----------+-------+------+-----------+--------------------+--------+----------|\n");

            for (int i = 0; i < (int)liste_v.size(); i += 1)
            {
                if( liste_v.at(i)->get_messages() > 1 )         // Pour filter les bétises...
                    if( liste_v.at(i)->last_update() < 600 )    // On n'affiche que les avions des 10 dernieres minutes
                        liste_v.at(i)->print();
            }
        }

        if ( (dump_resume == false) && (acq_counter % acq_per_sec == 0) )
        {
            switch (n_dots)
            {
                case 0: printf("         \r"); break;
                case 1: printf(".        \r"); break;
                case 2: printf("..       \r"); break;
                case 3: printf("...      \r"); break;
                case 4: printf("....     \r"); break;
                case 5: printf(".....    \r"); break;
                case 6: printf("......   \r"); break;
                case 7: printf(".......  \r"); break;
                case 8: printf("........ \r"); break;
                case 9: printf(".........\r"); break;
            }
            fflush(stdout);
            n_dots = (n_dots + 1) % 10;
        }
    }
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////////////


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

    //////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    dump_relatime.update_all( liste_v );
    //
    //////////////////////////////////////////////////////////////////////////////////////////////////

    //
    // Affichage des statistiques d'execution
    //

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    double RTConstr = 1000.0f / (param.toDouble("fe") / buffer.size());
    double avgTime = chrono::duration_cast<chrono::milliseconds>(elapsed).count() / (float) acq_counter;

    std::cout << "(II)" << std::endl;
    std::cout << "(II) Nombre d'aquisition réalisées  : " << acq_counter << std::endl;
    std::cout << "(II) Temps moyen par acquisition    : " << avgTime     << " ms" << std::endl;
    std::cout << "(II) Constrainte tps réel / iter    : " << RTConstr    << " ms" << std::endl;

    stats.dump();

    std::cout << "(II)" << std::endl;
    std::cout << "(II) Temps total : " << chrono::duration_cast<chrono::milliseconds>(end - start).count() << " ms" << std::endl;
    std::cout << "(II)" << std::endl;

    //
    // Si on a loggué les données dans des fichiers, il est temps de les fermer proprement...
    //

    if( StoreDataSet == true )
    {
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
