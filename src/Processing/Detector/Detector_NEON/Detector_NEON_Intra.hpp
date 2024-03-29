#ifndef _Detector_NEON_
#define _Detector_NEON_

#include "../Detector.hpp"
#if defined(__ARM_NEON)
    #include <arm_neon.h>
#endif


class Detector_NEON_Intra : public Detector{
private :
    virtual void execute(float* buffer);

public :
	Detector_NEON_Intra();

    virtual void execute(std::vector<float>* iBuffer, std::vector<float>* oBuffer);

};

#endif
