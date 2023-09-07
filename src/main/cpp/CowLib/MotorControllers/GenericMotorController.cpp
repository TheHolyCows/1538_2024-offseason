#include "GenericMotorController.h"

namespace CowLib
{

    GenericMotorController::GenericMotorController(int id, std::string bus, CowMotorUtils::MotorType motorType)
    {
        printf("GenericMotorController(id=%i, bus=%s, motorType=%i)\n",id,bus.c_str(),motorType);
    }
}