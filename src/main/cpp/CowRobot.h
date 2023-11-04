//==================================================
// Copyright (C) 2018 Team 1538 / The Holy Cows
//==================================================

#ifndef __COW_ROBOT_H__
#define __COW_ROBOT_H__

#include "Controllers/GenericController.h"
#include "CowConstants.h"
#include "CowLib/CowAlphaNum.h"
#include "CowLib/CowLogger.h"
#include "CowLib/CowMotorController.h"
#include "CowLib/CowPID.h"
#include "CowLib/CowTimer.h"
#include "CowLib/Utility.h"
#include "CowPigeon.h"
#include "frc/controller/PIDController.h"
#include "Subsystems/Vision.h"
#include "Driverain/SwerveDrive.h"

#include <frc/BuiltInAccelerometer.h>
#include <frc/filter/LinearFilter.h>
#include <frc/PowerDistribution.h>
#include <math.h>
#include <vector>

class CowRobot
{
public:

private:

    int m_DSUpdateCount;

    GenericController *m_Controller = nullptr;

    // TODO: add drivetrain member variable
    SwerveDrive *m_Drivetrain; 
     


    // gyro and accelerometers
    CowPigeon *m_Gyro;
    frc::Accelerometer *m_Accelerometer;
    frc::LinearFilter<double> m_ZFilter = frc::LinearFilter<double>::MovingAverage(12);
    double m_PrevZ;

    // PDP
    frc::PowerDistribution *m_PowerDistributionPanel;

    // display on rio removed
    CowLib::CowAlphaNum *m_LEDDisplay;

    double m_LeftDriveValue;
    double m_RightDriveValue;

    double m_PreviousGyroError;
    double m_PreviousDriveError;

    double m_MatchTime;
    double m_StartTime;

public:
    CowRobot();
    void Reset();
    void SetController(GenericController *controller);
    void PrintToDS();

    void StartTime();

    CowLib::CowAlphaNum *GetDisplay() { return m_LEDDisplay; }

    frc::PowerDistribution *GetPowerDistributionPanel() { return m_PowerDistributionPanel; }

    CowPigeon *GetGyro() { return CowPigeon::GetInstance(); }

    // TODO: return the drivetrain
    SwerveDrive *GetDrivetrain() { return m_Drivetrain; }

    // TODO: return the drive controller
    SwerveDriveController *GetDriveController() { return m_DriveController; };

    void Handle();

    void DoNothing(void);
};

#endif
