//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
//==================================================

#ifndef __COWLIB_COWMOTORCONTROLLER_H__
#define __COWLIB_COWMOTORCONTROLLER_H__

#include <ctre/phoenixpro/TalonFX.hpp>
#include <variant>
#include "CowMotorUtilities.h"

namespace CowLib
{
    class CowMotorController
    {
    private:
        ctre::phoenixpro::hardware::TalonFX *m_Talon;
        double m_Setpoint;
        bool m_UseFOC;
        bool m_OverrideBrakeMode;

    public:
        CowMotorController(int id, std::string bus = "cowbus", CowMotorUtils::MotorType motorType = CowMotorUtils::PHOENIX_PRO);

        ~CowMotorController();

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

        void GetPIDData(double *setpoint, double *procVar, double *P, double *I, double *D);
        void GetLogData(double *temp, double *encoderCt, bool *isInverted);
    };
} // namespace CowLib

#endif /* __COWLIB_COWMOTORCONTROLLER_H__ */
