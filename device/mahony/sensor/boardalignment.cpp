#include "boardalignment.hpp"
namespace bfimu {
bool Alignment::standardBoardAlignment = true;
Matrix33 Alignment::boardRotation;
float Alignment::rollDegrees = DEFAULT_ALIGN_BOARD_ROLL;
float Alignment::pitchDegrees = DEFAULT_ALIGN_BOARD_PITCH;
float Alignment::yawDegrees = DEFAULT_ALIGN_BOARD_YAW;
}