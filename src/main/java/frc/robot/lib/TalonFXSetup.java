// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class TalonFXSetup {
    
    public static void defaultSetup(TalonFX motor, boolean isInverted, NeutralModeValue neutralMode, double currentLimit){
        motor.getConfigurator().apply(new TalonFXConfiguration());
        // motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        // motor.configVoltageCompSaturation(voltageCompSaturation);  //default 12v voltage compensation for motors
        // motor.enableVoltageCompensation(true);  //enable voltage compensation
        simpleCurrentLimit(motor, currentLimit);
        motor.setInverted(isInverted);
        motor.setNeutralMode(neutralMode);
        // motor.setSensorPhase(phaseSensor);
        // defaultStatusFrames(motor);
    }

    //Talon FX motor
    //Limit in Amps
    public static void simpleCurrentLimit(TalonFX motor, double limit) {
        CurrentLimitsConfigs simpleCurrentLimit =new CurrentLimitsConfigs();
        simpleCurrentLimit.SupplyCurrentLimitEnable = true;
        simpleCurrentLimit.SupplyCurrentLimit = limit;
        simpleCurrentLimit.SupplyCurrentThreshold = limit;
        simpleCurrentLimit.SupplyTimeThreshold = 0.5;
        // SupplyCurrentLimitConfiguration supplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, limit, limit, 0.5);
        motor.getConfigurator().refresh(simpleCurrentLimit);
    }

    // public static void defaultStatusFrames(TalonFX motor){
    //     //Default Status Rates are listed here: https://docs.ctre-phoenix.com/en/stable/ch18_CommonAPI.html
    //     int fastTime = 200;
    //     int slowTime = 250;
    //     motor.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
    //     motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 50);
    //     motor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, slowTime);
    //     motor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, fastTime);
    //     motor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, slowTime);
    //     motor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, fastTime);
    //     motor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, slowTime);
    //     motor.setStatusFramePeriod(StatusFrame.Status_17_Targets1, slowTime);
    // }

    // public static void velocityStatusFrames(TalonFX motor){
    //     int fastTime = 160;
    //     int slowTime = 200;
    //     motor.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
    //     motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
    //     motor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, slowTime);
    //     motor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, fastTime);
    //     motor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, slowTime);
    //     motor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, fastTime);
    //     motor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, slowTime);
    //     motor.setStatusFramePeriod(StatusFrame.Status_17_Targets1, slowTime);
    // }
}
