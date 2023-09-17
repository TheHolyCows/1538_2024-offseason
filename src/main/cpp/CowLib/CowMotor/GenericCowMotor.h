//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// @author jon-bassi
//==================================================

#pragma once

#include <string>
#include <variant>

namespace CowMotor
{   
    /**
     * @brief template class for all motor controllers
     */
    class GenericCowMotor
    {
        public:
        GenericCowMotor();
        GenericCowMotor(int id, std::string bus);

        /* remainder of the class is pure virtual */

        /* control requests */
        virtual void Set(std::variant<PercentOutput,
                                      VoltageOutput,
                                      PositionPercentOutput,
                                      PositionVoltage,
                                      VelocityPercentOutput,
                                      VelocityVoltage,
                                      MotionMagicPercentOutput,
                                      MotionMagicVoltage> request) = 0;
        virtual void Set(std::variant<TorqueCurrentOutput, 
                                      PositionTorqueCurrent,
                                      VelocityTorqueCurrent, 
                                      MotionMagicTorqueCurrent> request) = 0;
        virtual void Set(Follower request) = 0;

        /* configuration */
        virtual void UseFOC(bool useFOC) = 0;
        virtual void OverrideBrakeMode(bool overrideBrakeMode) = 0;
        virtual void ApplyConfig(std::variant<ctre::phoenixpro::configs::TalonFXConfiguration,
                                              ctre::phoenixpro::configs::Slot0Configs,
                                              ctre::phoenixpro::configs::MotionMagicConfigs,
                                              ctre::phoenixpro::configs::MotorOutputConfigs> config) = 0;

        /* getters */
        virtual double GetPosition() = 0;
        virtual double GetVelocity() = 0;
        virtual double GetTorqueCurrent() = 0;
        virtual double GetRefreshTorqueCurrent() = 0;
        virtual CowMotor::NeutralMode GetNeutralMode() = 0;

        /* setters */
        virtual int SetSensorPosition(double turns) = 0;
        virtual void SetNeutralMode(CowMotor::NeutralMode mode) = 0;
        virtual void SetPID(double p, double i, double d, double f = 0.0) = 0;
        virtual void SetMotionMagic(double velocity, double acceleration) = 0;
        virtual void SetInverted(bool inverted) = 0;
        virtual void SetReversed(bool reversed) = 0;

        // ctre::phoenixpro::hardware::TalonFX *GetInternalTalon();

        /* logging */
        virtual void GetPIDData(double *setpoint, double *procVar, double *P, double *I, double *D) = 0;
        virtual void GetLogData(double *temp, double *encoderCt, bool *isInverted) = 0;
    };
}