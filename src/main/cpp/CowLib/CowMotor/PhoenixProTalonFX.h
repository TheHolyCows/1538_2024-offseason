//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// @author jon-bassi
//==================================================

#pragma once

#include "GenericCowMotor.h"
#include "CowMotorUtils.h"

#include <ctre/phoenixpro/TalonFX.hpp>
#include <variant>


namespace CowMotor
{
    class PhoenixProTalonFX : public GenericCowMotor
    {
    public:
        PhoenixProTalonFX(int id, std::string bus);

        void Set(std::variant<PercentOutput,
                              VoltageOutput,
                              PositionPercentOutput,
                              PositionVoltage,
                              VelocityPercentOutput,
                              VelocityVoltage,
                              MotionMagicPercentOutput,
                              MotionMagicVoltage> request);

        void Set(std::variant<TorqueCurrentOutput, 
                              PositionTorqueCurrent, 
                              VelocityTorqueCurrent, 
                              MotionMagicTorqueCurrent> request);

        void Set(Follower request);

    private:
        ctre::phoenixpro::hardware::TalonFX *m_Talon;
        
        double m_Setpoint;
        bool m_UseFOC;
        bool m_OverrideBrakeMode;
    };
}