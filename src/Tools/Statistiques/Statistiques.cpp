#include "Statistiques.hpp"

 Statistiques::Statistiques( )
{
    start = std::chrono::steady_clock::now();
    nbTramesDetectees = 0;
    nbBonsCRCs        = 0;
    nbBonsCRC_init    = 0;
    nbBonsCRCs_1x     = 0;
    nbBonsCRCs_2x     = 0;
    nbBonsCRCs_3x     = 0;
    nbDF18Frames      = 0;
    nbStrangeFrames   = 0;
}

int  Statistiques::validated_crc         () { return (nbBonsCRC_init+nbBonsCRCs_1x+nbBonsCRCs_2x+nbBonsCRCs_3x); }
int  Statistiques::detected_frames       () { return nbTramesDetectees; }

void Statistiques::add_detected_frame    () { nbTramesDetectees++; }
void Statistiques::validated_crc_init    () { nbBonsCRC_init++;    }
void Statistiques::validated_crc_brute_1x() { nbBonsCRCs_1x++;     }
void Statistiques::validated_crc_brute_2x() { nbBonsCRCs_2x++;     }
void Statistiques::validated_crc_brute_3x() { nbBonsCRCs_3x++;     }
void Statistiques::add_type_18_frame     () { nbDF18Frames++;      }
void Statistiques::add_strange_frame     () { nbStrangeFrames++;   }

Statistiques::~Statistiques( )
{

}

void Statistiques::dump( )
{
    const auto stop  = std::chrono::steady_clock::now();
    const int eTime = std::chrono::duration_cast<std::chrono::seconds>(stop - start).count();
    std::cout << "(II)" << std::endl;
    std::cout << "(II) Application runtime : " << eTime << " seconds" << std::endl;
    std::cout << "(II) Number of frames over the detection value      : " << nbTramesDetectees << std::endl;
    std::cout << "(II) Number of frames with validated CRC value      : " << nbBonsCRCs        << std::endl;
    std::cout << "(II)  - Number of initially correct  CRC values     : " << nbBonsCRC_init    << std::endl;
    std::cout << "(II)  - Number of saved frames with bit-flip (x1)   : " << nbBonsCRCs_1x     << std::endl;
    std::cout << "(II)  - Number of saved frames with bit-flip (x2)   : " << nbBonsCRCs_2x     << std::endl;
    std::cout << "(II)  - Number of saved frames with bit-flip (x3)   : " << nbBonsCRCs_3x     << std::endl;
    std::cout << "(II) Number of discarded frames with DF = 18        : " << nbDF18Frames      << " (type == 18)" <<std::endl;
    std::cout << "(II) Number of discarded frames with strange values : " << nbStrangeFrames   << " (type != 17 && type != 18)" <<std::endl;
    std::cout << "(II)" << std::endl;
}
