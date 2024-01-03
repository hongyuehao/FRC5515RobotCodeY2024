// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import frc.robot.Constants;
import frc.robot.lib.SwerveMod;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveMod[] mSwerveMods;
    public Pigeon2 pigeon2;


    // double[] ypr = new double[3];
    //   public Pigeon pigeon;

    public Swerve() {
        pigeon2 = new Pigeon2(Constants.TeleopConstants.pigeonID);
        pigeon2.getConfigurator().apply(new Pigeon2Configuration());
        zeroGyro();
        
        swerveOdometry = new SwerveDriveOdometry(Constants.TeleopConstants.swerveKinematics, getYaw(), getModulePositions());

        mSwerveMods = new SwerveMod[] {
            new SwerveMod(0, Constants.TeleopConstants.Mod0.constants),
            new SwerveMod(1, Constants.TeleopConstants.Mod1.constants),
            new SwerveMod(2, Constants.TeleopConstants.Mod2.constants),
            new SwerveMod(3, Constants.TeleopConstants.Mod3.constants)
        };
    }

    /* Used by SwerveTeleopCommand in Teleop */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.TeleopConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
            //SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.TeleopConstants.maxSpeedMetersPerSecond);

        for(SwerveMod mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }  

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        //SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.AutoConstants.kMaxSpeedMetersPerSecond);
        
        for(SwerveMod mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveMod mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void zeroGyro(){
        pigeon2.setYaw(0.0);
    }

    // public void zeroGyro(double reset){
    //     pigeno2.setYaw(reset);
    // }

    public void resetAngleToAbsolute() {        
        for(SwerveMod mod : mSwerveMods){
            mod.resetToAbsolute();;
        }
    }

    public double gyrogetAngle() {
        return 0.0;
    }

    public Rotation2d getYaw() {
        // return (Constants.isAutoEnd) ? Rotation2d.fromDegrees(gyrogetAngle() + Constants.autoInitAngle) : Rotation2d.fromDegrees(gyrogetAngle());
        
        return (Constants.isAutoEnd) ? Rotation2d.fromDegrees(pigeon2.getYaw().getValue() + Constants.autoInitAngle) : Rotation2d.fromDegrees(pigeon2.getYaw().getValue());
        // return (Constants.TeleopConstants.invertGyro) ? Rotation2d.fromDegrees(360 - pigeno2.getYaw()) : Rotation2d.fromDegrees(pigeno2.getYaw());
    }

    public double swerveSpeed() {
        return Math.abs(mSwerveMods[2].getState().speedMetersPerSecond);
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveMod mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getModulePositions());  

        for(SwerveMod mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }

        SmartDashboard.putNumber("Pose X", getPose().getX());
        SmartDashboard.putNumber("Pose Y", getPose().getY());
        SmartDashboard.putNumber("Pose R", pigeon2.getYaw().getValue());
        SmartDashboard.putNumber("Pose Pitch", Constants.mRobotPitch);
        SmartDashboard.putNumber("Roll Direc", Constants.direc);
        SmartDashboard.putNumber("Roll min", Constants.minRoll);
        SmartDashboard.putNumber("Roll max", Constants.maxRoll);
        // SmartDashboard.putNumber("DriveTrain Angle", driveTrainAngle());
        // SmartDashboard.putNumber("Swerve Angle", swerveAngle());
    }
}
