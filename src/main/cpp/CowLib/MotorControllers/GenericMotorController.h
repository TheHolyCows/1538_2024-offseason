//==================================================
// Copyright (C) 2024 Team 1538 / The Holy Cows
//==================================================

#pragma once

#include <stdio.h>
#include <variant>
#include <ctre/phoenixpro/TalonFX.hpp>

#include "../CowMotorUtilities.h"

namespace CowLib
{
    class GenericMotorController
    {
        GenericMotorController(int id, std::string bus = "cowbus", CowMotorUtils::MotorType motorType = CowMotorUtils::PHOENIX_PRO);

        ~GenericMotorController();

        void Set(std::variant<CowMotorUtils::PercentOutput,
                                CowMotorUtils::VoltageOutput,
                                CowMotorUtils::PositionPercentOutput,
                                CowMotorUtils::PositionVoltage,
                                CowMotorUtils::VelocityPercentOutput,
                                CowMotorUtils::VelocityVoltage,
                                CowMotorUtils::MotionMagicPercentOutput,
                                CowMotorUtils::MotionMagicVoltage> request);

        void
        Set(std::variant<CowMotorUtils::TorqueCurrentOutput, CowMotorUtils::PositionTorqueCurrent, CowMotorUtils::VelocityTorqueCurrent, CowMotorUtils::MotionMagicTorqueCurrent>
                request);

        void Set(CowMotorUtils::Follower request);

        void UseFOC(bool useFOC);
        void OverrideBrakeMode(bool overrideBrakeMode);

        void ApplyConfig(std::variant<ctre::phoenixpro::configs::TalonFXConfiguration,
                                        ctre::phoenixpro::configs::Slot0Configs,
                                        ctre::phoenixpro::configs::MotionMagicConfigs,
                                        ctre::phoenixpro::configs::MotorOutputConfigs> config);

        double GetPosition();
        double GetVelocity();
        double GetTorqueCurrent();
        double GetRefreshTorqueCurrent();

        int SetSensorPosition(double turns);

        void SetNeutralMode(CowMotorUtils::NeutralMode mode);
        CowMotorUtils::NeutralMode GetNeutralMode();

        void SetPID(double p, double i, double d, double f = 0.0);
        void SetMotionMagic(double velocity, double acceleration);

        void SetInverted(bool inverted);

        ctre::phoenixpro::hardware::TalonFX *GetInternalTalon();
    };
}