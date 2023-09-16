#include "PhoenixV5TalonFX.h"

namespace CowMotor
{
    PhoenixV5TalonFX::PhoenixV5TalonFX(int id, std::string bus)
    {
        m_Talon = new ctre::phoenix::motorcontrol::can::TalonFX(id, std::move(bus));
        m_Setpoint = 0;
        m_OverrideBrakeMode = false;
    }

    /**
     * @brief from Talon code:
     * In PercentOutput, the output is between -1.0 and 1.0, with 0.0 as stopped.
     * In Current mode, output value is in amperes. (not setup currently)
     * In Velocity mode, output value is in rotations / 100ms.
     * In Position mode, output value is in rotations
     * In MotionMagic mode, output value is position of motor in rotations
     * In Follower mode, the output value is the integer device ID of the talon to
     *   duplicate. (below)
     * 
     * @param request 
     */
    void PhoenixV5TalonFX::Set(std::variant<PercentOutput,
                                              VoltageOutput,
                                              PositionPercentOutput,
                                              PositionVoltage,
                                              VelocityPercentOutput,
                                              VelocityVoltage,
                                              MotionMagicPercentOutput,
                                              MotionMagicVoltage> request)
    {
        auto &talon       = m_Talon;
        double *setpoint  = &m_Setpoint;
        auto &falcon_units_per_rot = FALCON_UNITS_PER_ROTATION;
        visit([talon, setpoint, falcon_units_per_rot](auto &&req)
        {
            ctre::phoenix::motorcontrol::TalonFXControlMode controlMode = req.GetControlMode();
            switch (controlMode)
            {
                case ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput :
                    talon->Set(controlMode,req.GetSetpoint());
                    *setpoint = req.GetSetpoint();
                    break;
                case ctre::phoenix::motorcontrol::TalonFXControlMode::Position :
                    talon->Set(controlMode,req.GetSetpoint() * falcon_units_per_rot);
                    *setpoint = req.GetSetpoint() * falcon_units_per_rot;
                    break;
                case ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity :
                    talon->Set(controlMode,req.GetSetpoint() * falcon_units_per_rot);
                    *setpoint = req.GetSetpoint() * falcon_units_per_rot;
                    break;
                case ctre::phoenix::motorcontrol::TalonFXControlMode::MotionMagic :
                    talon->Set(controlMode,req.GetSetpoint() * falcon_units_per_rot);
                    *setpoint = req.GetSetpoint() * falcon_units_per_rot;
                    break;
                case ctre::phoenix::motorcontrol::TalonFXControlMode::Disabled :
                    return;
            }
        },
        request);
        
    }

    /**
     * @brief none of these are supported for TalonFX on Phoenix v5
     * 
     * @param request 
     */
    void PhoenixV5TalonFX::Set(std::variant<TorqueCurrentOutput,
                                              PositionTorqueCurrent,
                                              VelocityTorqueCurrent,
                                              MotionMagicTorqueCurrent> request)
    {
        return;
    }

    void PhoenixV5TalonFX::Set(Follower request)
    {
        m_Talon->Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Follower,request.LeaderID);
        m_Setpoint = request.LeaderID;
    }
}