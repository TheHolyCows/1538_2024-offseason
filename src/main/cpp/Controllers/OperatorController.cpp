#include "OperatorController.h"

OperatorController::OperatorController(GenericControlBoard *controlboard)
    : m_CB(controlboard)
{
    m_TrackingCooldownTimer = 0.0;
}

void OperatorController::Handle(CowRobot *bot)
{   
    // X mode for wheels
    if (m_CB->GetDriveAxis(2) > 0.8 && m_CB->GetDriveAxis(6) > 0.8)
    {
        // bot->GetDrivetrain()->SetLocked(true);
        // bot->GetDrivetrain()->SetVelocity(0, 0, 0);
    }
    else
    {
        bot->GetDrivetrain()->SetLocked(false);
    }

    if (m_CB->GetVisionTargetButton())
    {
        // Vision::GetInstance()->...
    }
    else if (m_CB->GetDriveAxis(3) > 0.8) // heading lock for human player intake
    {
        // bot->GetDriveController()->LockHeading(m_CB->GetLeftDriveStickY(), m_CB->GetLeftDriveStickX());
    }
    else
    {
        // TODO: standard drive code here
        // we can get the drive axis of the control board through the m_CB object
        // we should drive through the use of the drive controller object in CowRobot
    }
}
