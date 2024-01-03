#pragma once

#include "../../CowLib/CowTimer.h"
#include "../../CowRobot.h"
#include "./RobotCommand.h"

#include <iostream>
#include <pathplanner/lib/commands/FollowPathHolonomic.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <string>
#include <vector>

class PathplannerSwerveTrajectoryCommand : public RobotCommand
{
public:
    struct Event
    {
        std::string waypointName;
        RobotCommand *command;
        double time  = -1;
        bool done    = false;
        bool started = false;
    };

    PathplannerSwerveTrajectoryCommand(const std::string &trajectoryName,
                                       double maxSpeed,
                                       double maxAccel,
                                       bool stop,
                                       bool resetOdometry        = false,
                                       std::vector<Event> events = {});
    ~PathplannerSwerveTrajectoryCommand() override;

    bool IsComplete(CowRobot *robot) override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;

    frc::Pose2d GetStartingPose();

private:
    CowLib::CowTimer *m_Timer;

    std::shared_ptr<pathplanner::PathPlannerPath> m_Path;
    pathplanner::FollowPathHolonomic *m_PathFollower;

    double m_TotalTime;
    bool m_Stop;
    bool m_ResetOdometry;

    std::vector<Event> m_Events;
};
