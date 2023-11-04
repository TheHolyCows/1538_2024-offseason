#include "CowRobot.h"

CowRobot::CowRobot()
{
    m_MatchTime     = 0;
    m_StartTime     = 0;
    m_DSUpdateCount = 0;

    // uncomment for b-bot
    m_PowerDistributionPanel = new frc::PowerDistribution(1, frc::PowerDistribution::ModuleType::kRev);
    // m_PowerDistributionPanel = new frc::PowerDistribution();

    // mxp board was removed from robot - can remove this code
    m_LEDDisplay = nullptr;

    m_Gyro = CowPigeon::GetInstance();

    m_PreviousGyroError = 0;
    // m_Gyro->Reset(); - don't know why we have this commented
    m_Accelerometer = new frc::BuiltInAccelerometer(frc::Accelerometer::kRange_4G);


    // TODO: initialize ModuleConstants
    // frontLeft, frontRight, backLeft, backRight
    // driveMotorID, turningMotorID, encoderCanId, moduleOffset
    // assume motor IDs are FL:(2,1), FR:(4,3), BL:(6,5), BR:(8,7)
    // assume encoderIds are 25, 26, 27, 28 respectively
    // assume module offset is 46.934, 28.125, 7.910, and 241.436 respectively
    SwerveDrive::ModuleConstants swerveModuleConstants[4]{
    {2,1,25,46.934},
    {4,3,26,28.125}, 
    {6,5,27,7.910},
    {8,7,28,241.436}
    };


    // TODO: initialize the SwerveDrive
    m_Drivetrain; = new SwerveDrive{swerveModuleConstants,1.5625}


    // TODO: intitialize the SwerveDriveController
    m_Drivetrain; = new SwerveDriveController{*m_DriveTrain}

}

/**
 * @brief reset drivetrain encoders and gyro
 */
void CowRobot::Reset()
{
    m_MatchTime = 0;

    m_PreviousGyroError = 0;

    CowLib::CowLogger::GetInstance()->Reset();
}

/**
 * @brief
 *
 * @param controller
 */
void CowRobot::SetController(GenericController *controller)
{
    m_Controller = controller;
}

void CowRobot::PrintToDS()
{
    if (m_DSUpdateCount++ % 10 == 0)
    {
        m_DSUpdateCount = 1;
    }
}

// Used to handle the recurring logic funtions inside the robot.
// Please call this once per update cycle.
void CowRobot::Handle()
{
    m_MatchTime = CowLib::CowTimer::GetFPGATimestamp() - m_StartTime;

    if (m_Controller == nullptr)
    {
        printf("No controller for CowRobot!!\n");
        return;
    }

    m_Controller->Handle(this);

    // logger code below should have checks for debug mode before sending out data
    CowLib::CowLogger::GetInstance()->Handle();

    PrintToDS();
}

void CowRobot::StartTime()
{
    m_StartTime = CowLib::CowTimer::GetFPGATimestamp();
}

void CowRobot::DoNothing()
{
    // TODO: make the robot stop (including drive)
}
