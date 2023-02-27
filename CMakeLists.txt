cmake_minimum_required(VERSION 3.5)

SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_BUILD_TYPE Release)

#SET (CMAKE_C_COMPILER   /usr/bin/clang)
#SET (CMAKE_CXX_COMPILER /usr/bin/clang++)

set(CMAKE_CXX_FLAGS "                   -Wall -Ofast -mcpu=native -mtune=native -Wno-unknown-pragmas")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Rpass=loop-vectorize")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/usr/local/include")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/opt/homebrew/Cellar/uhd/4.4.0.0/include")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/opt/homebrew/Cellar/hackrf/2023.01.1/include")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/opt/homebrew/Cellar/soapysdr/0.8.1_1/include")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/opt/homebrew/Cellar/boost/1.81.0_1/include")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/opt/homebrew/opt/bzip2/include")

# x86 part
set(CMAKE_EXE_LINKER_FLAGS "-L/usr/local/lib") # UHD

#set(CMAKE_EXE_LINKER_FLAGS "                          /usr/local/lib/libboost_filesystem-mt.dylib") # UHD
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /usr/local/lib/libboost_system-mt.dylib") # UHD
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /usr/local/opt/hackrf/lib/libhackrf.a") # UHD
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /usr/local/opt/libusb/lib/libusb-1.0.a") # UHD
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /usr/local/Cellar/uhd/4.1.0.3/lib/libuhd.dylib") # UHD
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -framework CoreFoundation -framework IOKit") # UHD
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lncurses") # UHD
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L/usr/local/lib -lSoapySDR") # SoapySDR

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -luhd") # UHD
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lhackrf") # UHD
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lSoapySDR") # SoapySDR
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L/opt/homebrew/Cellar/uhd/4.4.0.0/lib") # UHD
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L/opt/homebrew/Cellar/hackrf/2023.01.1/lib") # UHD
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L/opt/homebrew/Cellar/soapysdr/0.8.1_1/lib") # UHD
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L/opt/homebrew/opt/bzip2/lib") # UHD

set(CMAKE_CXX_STANDARD 17)


########################################################################################################################

project(rcv_adsb)

set(SOURCE_FILES
        ./src/Processing/CRC32b/CRC/CRC32b.cpp
        ./src/RingBuff/RingBuff.cpp

        ./src/Processing/CplxModule/CplxModule.cpp
        ./src/Processing/CplxModule/CplxModule_x86/CplxModule_x86.cpp
        ./src/Processing/CplxModule/CplxModule_NEON/CplxModule_NEON.cpp
        ./src/Processing/CplxModule/CplxModule_AVX2/CplxModule_AVX2.cpp
        ./src/Processing/CplxModule/Library/CplxModuleLibrary.cpp

        ./src/Radio/Receiver/Receiver.cpp
        ./src/Radio/Receiver/Radio/SoapyRTLSdr/ReceiverSoapyRTLSdr.cpp
        ./src/Radio/Receiver/Radio/SoapyHackRF/ReceiverSoapyHackRF.cpp
        ./src/Radio/Receiver/Radio/SoapyUHD/ReceiverSoapyUHD.cpp
        ./src/Radio/Receiver/Radio/HackRF/ReceiverHackRF.cpp
        ./src/Radio/Receiver/Radio/USRP/ReceiverUSRP.cpp
        ./src/Radio/Receiver/Radio/ThreadUSRP/ReceiverThreadUSRP.cpp
        ./src/Radio/Receiver/Radio/USRP/RcvUSRP_b100.cpp
        ./src/Radio/Receiver/Radio/ThreadUSRP/RcvThreadUSRP_b100.cpp

        ./src/Radio/Receiver/File/ReceiverFileRAW.cpp
        ./src/Radio/Receiver/File/ReceiverFileStreamRAW.cpp
        ./src/Radio/Receiver/File/ReceiverFileUHD.cpp
        ./src/Radio/Receiver/File/ReceiverFileBZ2.cpp
        ./src/Radio/Receiver/Library/ReceiverLibrary.cpp

        ./src/brute_force/brute_force_1x.cpp
        ./src/brute_force/brute_force_2x.cpp
        ./src/brute_force/brute_force_3x.cpp

        ./src/Export/ExportRealTime.cpp

        src/Processing/PPM/PPM_demod.cpp
        src/Processing/DataPacking/BitPacking.cpp
        ./src/Processing/CRC32b/CheckCRC32b/CheckCRC32b.cpp
        ./src/Processing/CRC32b/RemoveCRC32b/RemoveCRC32b.cpp
        src/Processing/ADSBSynchro/RemoveADSBSynchro.cpp
        src/Processing/IQ/IQ_Removing.cpp
        src/Processing/Sampling/DownSampling.cpp
        ./src/Processing/Detector/Detector.cpp
        ./src/Processing/Detector/Detector_x86/Detector_x86.cpp
        ./src/Processing/Detector/Detector_NEON/Detector_NEON_Intra.cpp
        ./src/Processing/Detector/Detector_NEON/Detector_NEON_Inter.cpp
        ./src/Processing/Detector/Detector_NEON/Detector_NEON_Accu.cpp
        ./src/Processing/Detector/Detector_AVX2/Detector_AVX2.cpp
        ./src/Processing/Detector/Library/DetectorLibrary.cpp

        ./src/Tools/Avion/Avion.cpp
        ./src/Tools/Avion/type_aircraft.cpp

        ./src/Tools/Parameters/Parameters.cpp
        ./src/Tools/CTickCounter/CTickCounter.cpp

        ./src/Tools/Conversion/cvt_float_i8.cpp
        ./src/Tools/Conversion/cvt_float_u8.cpp
        ./src/Tools/Conversion/cvt_u8_float.cpp
        ./src/Tools/Conversion/cvt_i8_float.cpp

        ./src/rcv_adsb.cpp
        )

add_executable(rcv_adsb ${SOURCE_FILES})

target_link_libraries(rcv_adsb uhd)
target_link_libraries(rcv_adsb hackrf)
target_link_libraries(rcv_adsb SoapySDR)
target_link_libraries(rcv_adsb bz2)
target_link_libraries(rcv_adsb pthread)
