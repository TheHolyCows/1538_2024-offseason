//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// @author jon-bassi
//==================================================

#pragma once

#include "GenericCowMotor.h"
#include "CowMotorUtils.h"

#include <ctre/Phoenix.h>
#include <variant>


namespace CowMotor
{
    class PhoenixV5TalonFX : public GenericCowMotor
    {
    public:
        PhoenixV5TalonFX(int id, std::string bus);

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
        int FALCON_UNITS_PER_ROTATION = 2048;

        ctre::phoenix::motorcontrol::can::TalonFX *m_Talon;
        double m_Setpoint;
        bool m_OverrideBrakeMode;
    };
}