// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveTeleop extends Command {

  private double rotation;
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;  
  private Swerve m_swerve;
  private XboxController controller;
  private int frontBackAxis;
  private int leftRightAxis;
  private int rotationAxis;
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  public SwerveTeleop(Swerve m_swerve, XboxController controller, int frontBackAxis, int leftRightAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
    this.m_swerve = m_swerve;
    addRequirements(m_swerve);

    this.controller = controller;
    this.frontBackAxis = frontBackAxis;
    this.leftRightAxis = leftRightAxis;
    this.rotationAxis = rotationAxis;
    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;
}

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double yAxis = -m_xspeedLimiter.calculate(controller.getRawAxis(frontBackAxis));
    double xAxis = -m_yspeedLimiter.calculate(controller.getRawAxis(leftRightAxis));
    double rAxis = -m_rotLimiter.calculate(controller.getRawAxis(rotationAxis));

    //test moudle angle
    // double yAxis = 0.15;
    // double xAxis = 0.0;
    // double rAxis = 0.0;
    
    /* Deadbands */
    // yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0.0 : yAxis * Constants.mDriveSpeedKp;
    // xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0.0 : xAxis * Constants.mDriveSpeedKp;
    // rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0.0 : rAxis * Constants.mDriveSpeedKp;

    /* Deadbands - new version */
    yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0.0 : (yAxis - Constants.stickDeadband * Math.signum(yAxis)) * Constants.mDriveSpeedKp;
    xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0.0 : (xAxis - Constants.stickDeadband * Math.signum(xAxis)) * Constants.mDriveSpeedKp;
    rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0.0 : (rAxis - Constants.stickDeadband * Math.signum(rAxis)) * Constants.mDriveSpeedKp;

    translation = new Translation2d(yAxis, xAxis).times(Constants.TeleopConstants.maxSpeedMetersPerSecond);
    rotation = rAxis * Constants.TeleopConstants.maxAngularVelocity;
    m_swerve.drive(translation, rotation, fieldRelative, openLoop);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
