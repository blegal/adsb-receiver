#ifndef _Detector_AVX2_Intra_
#define _Detector_AVX2_Intra_

#include "../Detector.hpp"


class Detector_AVX2_Intra : public Detector{
private :

public :
	Detector_AVX2_Intra();

    virtual void execute(float* buffer);

    virtual void execute(std::vector<float>* iBuffer, std::vector<float>* oBuffer);
};

#endif
