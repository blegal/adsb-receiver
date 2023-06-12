#ifndef _Detector_AVX2_Inter_
#define _Detector_AVX2_Inter_

#include "../Detector.hpp"


class Detector_AVX2_Inter : public Detector{
private :

public :
	Detector_AVX2_Inter();

    virtual void execute(std::vector<float>* iBuffer, std::vector<float>* oBuffer);
};

#endif
