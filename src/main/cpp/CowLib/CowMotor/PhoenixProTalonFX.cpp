#include "PhoenixProTalonFX.h"

namespace CowMotor
{
    PhoenixProTalonFX::PhoenixProTalonFX(int id, std::string bus)
    {
        m_Talon = new ctre::phoenixpro::hardware::TalonFX(id, std::move(bus));
        m_Setpoint          = 0;
        m_UseFOC            = true;
        m_OverrideBrakeMode = false;
    }


    void PhoenixProTalonFX::Set(std::variant<PercentOutput,
                                              VoltageOutput,
                                              PositionPercentOutput,
                                              PositionVoltage,
                                              VelocityPercentOutput,
                                              VelocityVoltage,
                                              MotionMagicPercentOutput,
                                              MotionMagicVoltage> request)
    {
        auto &talon            = m_Talon;
        double *setpoint       = &m_Setpoint;
        bool useFOC            = m_UseFOC;
        bool overrideBrakeMode = m_OverrideBrakeMode;

        visit(
            [talon, setpoint, useFOC, overrideBrakeMode](auto &&req)
            {
                talon->SetControl(
                    req.ToControlRequest().WithEnableFOC(useFOC).WithOverrideBrakeDurNeutral(overrideBrakeMode));
                *setpoint = req.GetSetpoint();
            },
            request);
    }

    void PhoenixProTalonFX::Set(std::variant<TorqueCurrentOutput,
                                              PositionTorqueCurrent,
                                              VelocityTorqueCurrent,
                                              MotionMagicTorqueCurrent> request)
    {
        auto &talon      = m_Talon;
        double *setpoint = &m_Setpoint;
        visit(
            [talon, setpoint](auto &&req)
            {
                talon->SetControl(req.ToControlRequest());
                *setpoint = req.GetSetpoint();
            },
            request);
    }

    void PhoenixProTalonFX::Set(Follower request)
    {
        m_Talon->SetControl(request.ToControlRequest());
        m_Setpoint = request.LeaderID;
    }
}