#include "ReceiverHackRF.hpp"
#include <unistd.h>
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
typedef enum {
    HW_SYNC_MODE_OFF = 0,
    HW_SYNC_MODE_ON = 1,
} hw_sync_mode_t;
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
#define FREQ_ONE_MHZ (1000000ll)
#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
ReceiverHackRF::ReceiverHackRF(float s_fc, float s_fe, const float rcv_gain) : Receiver(s_fc, s_fe), buff(4 * s_fe )
{
    freq_hz   = s_fc;
    fech_hz   = s_fe;
    amplifier = false;
    antenna   = false;
    vga_gain  = 40; // 30;    // 40; : validated values for adsb-like comm.
    lna_gain  = 40; // 16;    // 32; : validated values for adsb-like comm.

    //
    // On veut une seconde de signal pour ne rien rater...
    //

    std::vector<std::string> modules;
    modules.push_back("0000000000000000088869dc334b441b");
    modules.push_back("0000000000000000f77c60dc255ca8c3");
    modules.push_back("000000000000000075b068dc317bae07");
    modules.push_back("000000000000000026b468dc33776d8f");
    modules.push_back("0000000000000000088869dc242e9d1b");
    modules.push_back("0000000000000000088869dc3362561b");
    modules.push_back("0000000000000000f77c60dc247c50c3");
    modules.push_back("0000000000000000f77c60dc29897cc3");
    modules.push_back("0000000000000000f77c60dc29876dc3");
    modules.push_back("0000000000000000f77c60dc297136c3");

    int result;
    result = hackrf_init();
    if( result != HACKRF_SUCCESS ) {
        fprintf(stderr, "hackrf_init() failed: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
        exit( EXIT_FAILURE );
    }

    for(uint32_t n = 0; n < modules.size(); n += 1)
    {
        result = hackrf_open_by_serial(modules.at(n).c_str(), &device);
        if( result == HACKRF_SUCCESS ) break;
    }

    if( result != HACKRF_SUCCESS ) {
        fprintf(stderr, "hackrf_open() failed: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
        exit( EXIT_FAILURE );
    }

    result = hackrf_set_hw_sync_mode(device, 0 ? HW_SYNC_MODE_ON : HW_SYNC_MODE_OFF);
    if( result != HACKRF_SUCCESS ) {
        fprintf(stderr, "hackrf_set_hw_sync_mode() failed: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
        exit( EXIT_FAILURE );
    }


    result = hackrf_set_freq(device, fc);
    if( result != HACKRF_SUCCESS ) {
        fprintf(stderr, "hackrf_set_freq() failed: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
        exit( EXIT_FAILURE );
    }

    result = hackrf_set_sample_rate(device, fe);
    if( result != HACKRF_SUCCESS ) {
        fprintf(stderr, "hackrf_set_sample_rate() failed: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
        exit( EXIT_FAILURE );
    }    

    const uint8_t control = 0; //amplifier ? 14 : 0;            // Pour activer l'amplificateur la valeur a fournir
    result = hackrf_set_amp_enable(device, (uint8_t)control);   // doit etre 14, 0 sinon...
    if( result != HACKRF_SUCCESS ) {
        fprintf(stderr, "hackrf_set_amp_enable() failed: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
        exit( EXIT_FAILURE );
    }

    result = hackrf_set_antenna_enable(device, (uint8_t)0/*antenna*/);
    if (result != HACKRF_SUCCESS) {
        fprintf(stderr, "hackrf_set_antenna_enable() failed: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
        exit( EXIT_FAILURE );
    }

    gain = ( rcv_gain == -1 ) ? 40 : rcv_gain;
    std::cout << KGRN << "[HackRF] " << KNRM << " Central frequency is set to  : " << s_fc << " Hz" << std::endl;
    std::cout << KGRN << "[HackRF] " << KNRM << " Sampling rate is set to      : " << s_fe << " Hz" << std::endl;
    std::cout << KGRN << "[HackRF] " << KNRM << " Receiver VGA gain is set to  : " << gain << " dB" << std::endl;
    std::cout << KGRN << "[HackRF] " << KNRM << " Receiver LNA gain is set to  : " << gain << " dB" << std::endl;

    result  = hackrf_set_vga_gain(device, (int)gain);
    if( result != HACKRF_SUCCESS ) {
        fprintf(stderr, "hackrf_set_vga_gain() failed: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
        exit( EXIT_FAILURE );
    }

    result = hackrf_set_lna_gain(device, (int)gain);
    if( result != HACKRF_SUCCESS ) {
        fprintf(stderr, "hackrf_set_lna_gain() failed: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
        exit( EXIT_FAILURE );
    }

    result = hackrf_start_rx(device, rx_callback, (void*)this);
    if( result != HACKRF_SUCCESS ) {
        fprintf(stderr, "hackrf_start_rx() failed: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
        exit( EXIT_FAILURE );
    }

    result = hackrf_is_streaming(device);
    if( result != HACKRF_TRUE ) {
        fprintf(stderr, "hackrf_start_rx() failed to check hackrf_is_streaming status: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
        exit( EXIT_FAILURE );
    }

/*
    set_freq          ( freq_hz   );
    set_sample_rate   ( fech_hz   );
    set_amp_enable    ( amplifier );
    set_antenna_enable( antenna   );
    set_vga_gain      ( vga_gain  );
    set_lna_gain      ( lna_gain  );
*/
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
ReceiverHackRF::~ReceiverHackRF()
{
    int result = hackrf_is_streaming(device);
    if( result != HACKRF_TRUE ) {
        fprintf(stderr, "hackrf_stop_rx() failed because device is not steraming: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
        exit( EXIT_FAILURE );
    }

    printf("(II) Receiver_HackRF::hackrf_stop_rx() is executed...\n");
    result = hackrf_stop_rx(device);
    if( result != HACKRF_SUCCESS )
    {
        fprintf(stderr, "hackrf_stop_rx() failed: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
        exit( EXIT_FAILURE );
    }

    hackrf_close( device );
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
int ReceiverHackRF::rx_callback(hackrf_transfer* transfer)
{
    ReceiverHackRF *obj = (ReceiverHackRF*) transfer->rx_ctx;
    return obj->rx_callback(transfer->buffer, transfer->valid_length);
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
int ReceiverHackRF::rx_callback(unsigned char *buf, uint32_t len)
{
//    const uint32_t start = ptr_write;
//    const uint32_t stop  = ptr_write + len;

    const uint32_t to_receive = buff.NumFreeElements();
    if( to_receive < len )
    {
        printf("(EE) Buffer overflow during reception...\n");
        exit( EXIT_FAILURE );
    }

    const uint32_t nWrite = buff.Write( (int8_t *)buf, len );
    if( nWrite != len )
    {
        printf("(EE) An error appear during the data writing in the buffer...\n");
        exit( EXIT_FAILURE );
    }else{
//        printf("(II) Data were received (len = %d - buffer = %d)...\n", len, buff.NumElements());
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
void ReceiverHackRF::initialize()
{

}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//void ReceiverHackRF::set_freq(double value)
//{
//    freq_hz = value;
//    fprintf(stderr, "call hackrf_set_freq(%f Hz/%.03f MHz)\n", freq_hz, ((double)freq_hz/(double)FREQ_ONE_MHZ) );
//    int32_t result = hackrf_set_freq(device, freq_hz);
//    if( result != HACKRF_SUCCESS ) {
//        fprintf(stderr, "hackrf_set_freq() failed: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
//        exit( EXIT_FAILURE );
//    }
//}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//double ReceiverHackRF::get_freq( )
//{
//    return freq_hz;
//}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//void ReceiverHackRF::set_sample_rate(double value)
//{
//    fech_hz = value;
//    fprintf(stderr, "call hackrf_set_sample_rate(%f Hz/%.03f MHz)\n", fech_hz, ((double)fech_hz/(double)FREQ_ONE_MHZ) );
//    int32_t result = hackrf_set_sample_rate(device, fe);
//    if( result != HACKRF_SUCCESS ) {
//        fprintf(stderr, "hackrf_set_sample_rate() failed: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
//        exit( EXIT_FAILURE );
//    }    
//}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//double ReceiverHackRF::get_sample_rate( )
//{
//    return fech_hz;
//}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//void ReceiverHackRF::set_amp_enable(bool value)
//{
//    fprintf(stderr, "call hackrf_set_amp_enable(%u => %u)\n", amplifier, value);
//    amplifier = value;
//    const uint8_t control = amplifier ? 14 : 0;                         // Pour activer l'amplificateur la valeur a fournir
//    int32_t result = hackrf_set_amp_enable(device, (uint8_t)control);   // doit etre 14, 0 sinon...
//    if( result != HACKRF_SUCCESS ) {
//        fprintf(stderr, "hackrf_set_amp_enable() failed: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
//        exit( EXIT_FAILURE );
//    }
//}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//bool ReceiverHackRF::get_amp_enable( )
//{
//    return amplifier;
//}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//void ReceiverHackRF::set_antenna_enable(bool value)
//{
//    fprintf(stderr, "call hackrf_set_antenna_enable(%u => %u)\n", antenna, value);
//    antenna = value;
//    int32_t result = hackrf_set_antenna_enable(device, (uint8_t)antenna);
//    if (result != HACKRF_SUCCESS) {
//        fprintf(stderr, "hackrf_set_antenna_enable() failed: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
//        exit( EXIT_FAILURE );
//    }
//}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//bool ReceiverHackRF::get_antenna_enable( )
//{
//    return antenna;
//}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//void ReceiverHackRF::set_vga_gain(uint32_t value)
//{
//    fprintf(stderr, "call hackrf_set_vga_gain(%u => %u)\n", vga_gain, value);
//    assert( value   >=  0 );
//    assert( value   <= 62 );
//    assert( value%2 == 0 );
//
//    vga_gain = value;
//    int32_t result  = hackrf_set_vga_gain(device, vga_gain);
//    if( result != HACKRF_SUCCESS ) {
//        fprintf(stderr, "hackrf_set_vga_gain() failed: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
//        exit( EXIT_FAILURE );
//    }
//}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//uint32_t ReceiverHackRF::get_vga_gain( )
//{
//    return vga_gain;
//}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//void ReceiverHackRF::set_lna_gain(uint32_t value)
//{
//    fprintf(stderr, "call hackrf_set_lna_gain(%u => %u)\n", lna_gain, value);
//    assert( value   >=  0 );
//    assert( value   <= 40 );
//    assert( value%8 ==  0 );
//
//    lna_gain = value;
//    int32_t result = hackrf_set_lna_gain(device, lna_gain);
//    if( result != HACKRF_SUCCESS ) {
//        fprintf(stderr, "hackrf_set_lna_gain() failed: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
//        exit( EXIT_FAILURE );
//    }
//}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//uint32_t ReceiverHackRF::get_lna_gain( )
//{
//    return lna_gain;
//}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
void ReceiverHackRF::start_engine()
{
//    int result = hackrf_start_rx(device, rx_callback, (void *)this);
//    if( result != HACKRF_SUCCESS ) {
//        fprintf(stderr, "hackrf_start_rx() failed: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
//        exit( EXIT_FAILURE );
//    }
//
//    result = hackrf_is_streaming(device);
//    if( result != HACKRF_TRUE ) {
//        fprintf(stderr, "hackrf_start_rx() failed to check hackrf_is_streaming status: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
//        exit( EXIT_FAILURE );
//    }
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
void ReceiverHackRF::stop_engine()
{
//    int result = hackrf_is_streaming(device);
//    if( result != HACKRF_TRUE ) {
//        fprintf(stderr, "hackrf_stop_rx() failed because device is not steraming: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
//        exit( EXIT_FAILURE );
//    }
//
//    result = hackrf_stop_rx(device);
//    if( result != HACKRF_SUCCESS )
//    {
//        fprintf(stderr, "hackrf_stop_rx() failed: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
//        exit( EXIT_FAILURE );
//    }
}
//
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
bool ReceiverHackRF::reception(std::vector< std::complex<float> >& cbuffer, const uint32_t coverage)
{
    //
    // On gere le vieillissement du buffer d'echantillons !
    //
    const uint32_t nOffset  = cbuffer.size() - coverage;
    for(uint32_t loop = 0; loop < coverage; loop += 1)
    {
        cbuffer[loop] = cbuffer[nOffset + loop];
    }

    int result = hackrf_is_streaming(device);
    if( result != HACKRF_TRUE ) {
        fprintf(stderr, "ReceiverHackRF::reception() failed because device is not steraming: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
        exit( EXIT_FAILURE );
    }

    const uint32_t toRead = 2 * (cbuffer.size() - coverage); // le buffer parle en bytes (et non en nombre de couples I/Q)
    if( toRead > buff.Capacity() )                           // N : est exrimé en nombre de bytes (et non en nombre de couples I/Q)
    {
        fprintf(stderr, "ReceiverHackRF::reception() failed: the number of data to read is higher than buffer size (%d)\n", toRead);
        exit( EXIT_FAILURE );
    }

    //
    // On se met en attente du bon nombre de données
    //
    while(buff.NumElements() < toRead)     // les 2 sont exprimés en nombre de samples
        usleep(1000); // queue empty

    int8_t* buf    = new int8_t[toRead];
    uint32_t nRead = buff.Read(buf, toRead);
    if( nRead != toRead )
    {
        fprintf(stderr, "ReceiverHackRF::reception() failed: the number of data to read is different than the resquest (%d)\n", toRead);
        exit( EXIT_FAILURE );
    }
/*
    for(uint32_t i = 0; i < toRead; i += 2)
    {
        std::complex<float> value( (float)buf[i] / 128.0f, (float)buf[i+1] / 128.0f );
        cbuffer[i/2 + coverage] = value;
    }
*/
    //
    //  Conversion et calcul du maximum
    //
    float maxv = 0.0f;
    float* ptr = (float*)&cbuffer[coverage];
    for(int i = 0; i < toRead; i += 1)
    {
        const float value = ((float)buf[i]) / 128.0f;
        ptr[i]            = value;
        maxv              = std::max(maxv, value);
    }
    IQ_max = maxv;

    delete[] buf;

    //
    //  fin de la conversion et calcul du maximum
    //


    //
    // suppression de l'offset des voies I/Q
    //

    float avgI = 0.0f;
    float avgQ = 0.0f;
    for(int i = 0; i < toRead; i += 2)
    {
        avgI += ptr[i + 0];
        avgQ += ptr[i + 1];
    }

    avgI /= (toRead / 2);
    avgQ /= (toRead / 2);

    for(int i = 0; i < toRead; i += 2)
    {
        ptr[i + 0] -= avgI;
        ptr[i + 1] -= avgQ;
    }

    //
    // fin de la suppression de l'offset des voies I/Q
    //

    return true;
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
void ReceiverHackRF::reset()
{
    fprintf(stderr, "ReceiverHackRF::reset() not implemented yet !\n");
    exit( EXIT_FAILURE );
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
float ReceiverHackRF::IQ_max_value()
{
    return IQ_max;
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
void ReceiverHackRF::IncreaseGain()
{
    gain = ( gain == 40 ) ? 40 : (gain + 2);

    int gain_vga = 2 * (gain / 2); // On prend en consideration les pas issus de la doc
    int gain_lna = 8 * (gain / 8); // On prend en consideration les pas issus de la doc

    int result  = hackrf_set_vga_gain(device, gain_vga);
    if( result != HACKRF_SUCCESS ) {
        fprintf(stderr, "hackrf_set_vga_gain() failed: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
        exit( EXIT_FAILURE );
    }

    result = hackrf_set_lna_gain(device, gain_lna);
    if( result != HACKRF_SUCCESS ) {
        fprintf(stderr, "hackrf_set_lna_gain() failed: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
        exit( EXIT_FAILURE );
    }
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
void ReceiverHackRF::DecreaseGain()
{
    gain = ( gain == 0 ) ? 0 : (gain - 2);

    int gain_vga = 2 * (gain / 2); // On prend en consideration les pas issus de la doc
    int gain_lna = 8 * (gain / 8); // On prend en consideration les pas issus de la doc

    int result  = hackrf_set_vga_gain(device, gain_vga);
    if( result != HACKRF_SUCCESS ) {
        fprintf(stderr, "hackrf_set_vga_gain() failed: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
        exit( EXIT_FAILURE );
    }

    result = hackrf_set_lna_gain(device, gain_lna);
    if( result != HACKRF_SUCCESS ) {
        fprintf(stderr, "hackrf_set_lna_gain() failed: %s (%d)\n", hackrf_error_name((hackrf_error)result), result);
        exit( EXIT_FAILURE );
    }
}
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//