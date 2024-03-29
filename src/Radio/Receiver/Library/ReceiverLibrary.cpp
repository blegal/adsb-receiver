#include "ReceiverLibrary.hpp"

#include "../File/ReceiverFileRAW.hpp"
#include "../File/ReceiverFileUHD.hpp"
#include "../File/ReceiverFileStreamRAW.hpp"
#include "../File/ReceiverFileBZ2.hpp"

#include "../Radio/HackRF/ReceiverHackRF.hpp"
#include "../Radio/USRP/ReceiverUSRP.hpp"
#include "../Radio/ThreadUSRP/ReceiverThreadUSRP.hpp"

#include "../Radio/USRP/RcvUSRP_b100.hpp"
#include "../Radio/ThreadUSRP/RcvThreadUSRP_b100.hpp"

#include <algorithm>

inline bool iequals(const string& a, const string& b)
{
    return std::equal(a.begin(), a.end(),
                      b.begin(), b.end(),
                      [](char a, char b) {
                          return tolower(a) == tolower(b);
                      });
}



ReceiverLibrary::ReceiverLibrary()
{

}


ReceiverLibrary::~ReceiverLibrary()
{

}

Receiver* ReceiverLibrary::allocate(Parameters& param)
{
    std::string type   = param.toString("mode_radio");
    std::string module = param.toString("filename");

    transform(type.begin(), type.end(), type.begin(), ::tolower);

    Receiver* radio;
    //
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    if(
            (type == "radio" && iequals(module, "hackrf"))
    ) {
        ReceiverHackRF* r = new ReceiverHackRF(param.toDouble("fc"), param.toDouble("fe"), param.toInt("receiver_gain"));
        //if( param.toInt("hackrf_amplifier") != -1 ) r->set_amp_enable( param.toInt("hackrf_amplifier") );
        //if( param.toInt("receiver_gain"   ) != -1 ) r->set_vga_gain  (  );
        //if( param.toInt("receiver_gain"   ) != -1 ) r->set_lna_gain  ( param.toInt("receiver_gain") );
        radio = r;

    //
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    } else if(
            (type == "radio" && (iequals(module, "usrp"))) ||
            (type == "radio" && (iequals(module, "usrp-b205")))
            ) {
        ReceiverUSRP* r = new ReceiverUSRP(param.toDouble("fc"), param.toDouble("fe"));
        if( param.toInt("receiver_gain")  != -1 )
            r->set_rx_gain( param.toInt("receiver_gain") );
        radio = r;

    } else if(
            (type == "radio" && iequals(module, "threadusrp") ) ||
            (type == "radio" && iequals(module, "thread_usrp")) ||
            (type == "radio" && iequals(module, "threadusrp-b205") ) ||
            (type == "radio" && iequals(module, "thread_usrp-b205"))
            ) {
        ReceiverThreadUSRP* r = new ReceiverThreadUSRP(param.toDouble("fc"), param.toDouble("fe"));
        if( param.toInt("receiver_gain")  != -1 )
            r->set_rx_gain( param.toInt("receiver_gain") );
        radio = r;

    } else if(
            type == "radio" && (iequals(module, "usrp-b100"))
            ) {
        RcvUSRP_b100* r = new RcvUSRP_b100(param.toDouble("fc"), param.toDouble("fe"));
        if( param.toInt("receiver_gain")  != -1 )
            r->set_rx_gain( param.toInt("receiver_gain") );
        radio = r;

    } else if(
            (type == "radio" && iequals(module, "threadusrp-b100") ) ||
            (type == "radio" && iequals(module, "thread_usrp-b100"))
            ) {
        RcvThreadUSRP_b100* r = new RcvThreadUSRP_b100(param.toDouble("fc"), param.toDouble("fe"));
        if( param.toInt("receiver_gain")  != -1 )
            r->set_rx_gain( param.toInt("receiver_gain") );
        radio = r;

    //
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    } else if( type == "file" && (module.find(".raw") != std::string::npos) ) {
        radio = new ReceiverFileRAW(param.toString("filename"));                    // SIGNED 8b

    } else if( type == "file" && (module.find(".cs8") != std::string::npos) ) {
        radio = new ReceiverFileRAW(param.toString("filename"));                    // SIGNED 8b

    } else if( type == "file" && (module.find(".cu8") != std::string::npos) ) {
        radio = new ReceiverFileRAW(param.toString("filename"), true);    // UNSIGNED 8b

    } else if( type == "file" && (module.find(".txt") != std::string::npos) ) {
        radio = new ReceiverFileUHD(param.toString("filename"));
    //
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    } else if( type == "file-stream" && (module.find(".raw") != std::string::npos) ) {
        ReceiverFileStreamRAW* r = new ReceiverFileStreamRAW(param.toString("filename"));
        radio = r;
    } else if( type == "file-stream" && (module.find(".cs8.bz2") != std::string::npos) ) {
        ReceiverFileBZ2* r = new ReceiverFileBZ2(param.toString("filename"), false);
        radio = r;
    } else if( type == "file-stream" && (module.find(".cu8.bz2") != std::string::npos) ) {
        ReceiverFileBZ2* r = new ReceiverFileBZ2(param.toString("filename"), true);
        radio = r;
    } else if( type == "file-stream" && (module.find(".cs8") != std::string::npos) ) {
        ReceiverFileStreamRAW* r = new ReceiverFileStreamRAW(param.toString("filename"), false);
        radio = r;
    } else if( type == "file-stream" && (module.find(".cu8") != std::string::npos) ) {
        ReceiverFileStreamRAW* r = new ReceiverFileStreamRAW(param.toString("filename"), true);
        radio = r;
    }
    //
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    else
    {
        cout << "Error in file (" << __FILE__ << ")" << " at line (" << __LINE__ << ")" << endl;
        cout << "mode_radio = " << type << endl;
        cout << "filename   = " << param.toString("filename")      << endl;
        exit( EXIT_FAILURE );
    }
    return radio;
}
