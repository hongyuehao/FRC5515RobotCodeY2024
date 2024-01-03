// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.TeleopConstants;

/** Swerve module */
public class SwerveMod {
    public int moduleNumber;
    private double angleOffset;
    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;
    private double lastAngle;

    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    private final PositionVoltage positionVoltage = new PositionVoltage(0);

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.TeleopConstants.driveKS, Constants.TeleopConstants.driveVelocity, Constants.TeleopConstants.driveKA);

    public SwerveMod(int moduleNumber, SwerveModConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond
                    / Constants.TeleopConstants.maxSpeedMetersPerSecond;
            dutyCycleOut.Output = percentOutput;
            mDriveMotor.setControl(dutyCycleOut);
        }
        else {
            double velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond,
                    Constants.TeleopConstants.wheelCircumference, Constants.TeleopConstants.driveGearRatio);
            velocityVoltage.Velocity = velocity;
            velocityVoltage.FeedForward = feedforward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(velocityVoltage);
        }

        double angle = (Math
                .abs(desiredState.speedMetersPerSecond) <= (Constants.TeleopConstants.maxSpeedMetersPerSecond
                        * Constants.driveAngleDeadband)) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        positionVoltage.Position = Conversions.degreesToRotations(angle, TeleopConstants.angleGearRatio);
        mAngleMotor.setControl(positionVoltage);
        lastAngle = angle;
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset, Constants.TeleopConstants.angleGearRatio);
        mAngleMotor.setPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());        
        angleEncoder.getConfigurator().apply(CTREConfigs.swerveCancoderConfig());
    }

    private void configAngleMotor(){
        mAngleMotor.getConfigurator().apply(new TalonFXConfiguration());
        mAngleMotor.getConfigurator().apply(CTREConfigs.swerveAngleFXConfig());
        mAngleMotor.setInverted(Constants.TeleopConstants.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.TeleopConstants.angleNeutralMode);
        // TalonFXSetup.defaultStatusFrames(mAngleMotor);
        resetToAbsolute();
    }

    private void configDriveMotor(){
        mDriveMotor.getConfigurator().apply(new TalonFXConfiguration());
        mDriveMotor.getConfigurator().apply(CTREConfigs.swerveDriveFXConfig());
        // mDriveMotor.setInverted(TalonFXInvertType.Clockwise);
        mDriveMotor.setNeutralMode(Constants.TeleopConstants.driveNeutralMode);
        mDriveMotor.setPosition(0);
        // TalonFXSetup.defaultStatusFrames(mDriveMotor);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public double getTargetAngle() {
        return lastAngle;
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(),
                Constants.TeleopConstants.wheelCircumference, Constants.TeleopConstants.driveGearRatio);
        Rotation2d angle = Rotation2d.fromRotations(mAngleMotor.getPosition().getValue());
        return new SwerveModuleState(velocity, angle);
    }
    
    public SwerveModulePosition getPosition() {
        double position = Conversions.rotationToMeters(mDriveMotor.getPosition().getValue(),
                Constants.TeleopConstants.wheelCircumference, Constants.TeleopConstants.driveGearRatio);
        Rotation2d angle = Rotation2d.fromRotations(mAngleMotor.getPosition().getValue());
        return new SwerveModulePosition(position, angle);
    }
}
