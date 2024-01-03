// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;

public final class CTREConfigs {

    public static TalonFXConfiguration swerveDriveFXConfig() {
        TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
        /* Swerve Drive Motor Configuration */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.TeleopConstants.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.TeleopConstants.driveContinuousCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.TeleopConstants.drivePeakCurrentDuration;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.TeleopConstants.drivePeakCurrentDuration;

        swerveDriveFXConfig.MotorOutput.PeakForwardDutyCycle = Constants.mDriveSpeedKp;
        swerveDriveFXConfig.MotorOutput.PeakReverseDutyCycle = -Constants.mDriveSpeedKp;
        swerveDriveFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // swerveDriveFXConfig.voltageCompSaturation = Constants.CommonConstants.mSwerveDriveFX_VoltageCompSaturation;
        swerveDriveFXConfig.Slot0.kP = Constants.TeleopConstants.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.TeleopConstants.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.TeleopConstants.driveKD;
        swerveDriveFXConfig.Slot0.kV = Constants.TeleopConstants.driveKV;        
        // swerveDriveFXConfig.CurrentLimits = driveSupplyLimit;
        // swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.TeleopConstants.openLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.TeleopConstants.closedLoopRamp;
        return swerveDriveFXConfig;
    }

    public static TalonFXConfiguration swerveAngleFXConfig() {
        TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
        /* Swerve Angle Motor Configurations */

        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.TeleopConstants.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.TeleopConstants.angleContinuousCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.TeleopConstants.anglePeakCurrentDuration;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.TeleopConstants.anglePeakCurrentDuration;

        swerveAngleFXConfig.MotorOutput.PeakForwardDutyCycle = Constants.mDriveSpeedKp;
        swerveAngleFXConfig.MotorOutput.PeakReverseDutyCycle = -Constants.mDriveSpeedKp;
        // swerveAngleFXConfig.voltageCompSaturation = Constants.CommonConstants.mSwerveAngleFX_VoltageCompSaturation;
        swerveAngleFXConfig.Slot0.kP = Constants.TeleopConstants.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.TeleopConstants.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.TeleopConstants.angleKD;
        swerveAngleFXConfig.Slot0.kV = Constants.TeleopConstants.angleKV;
        // swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        // swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        return swerveAngleFXConfig;
    }

    public static CANcoderConfiguration swerveCancoderConfig() {
        CANcoderConfiguration swerveCanCoderConfig = new CANcoderConfiguration();        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        // swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        // swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        return swerveCanCoderConfig;
    }

}