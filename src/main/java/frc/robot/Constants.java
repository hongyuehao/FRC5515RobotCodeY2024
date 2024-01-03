// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.SwerveModConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
  public static final int kTimeoutMs = 30; //use for on the fly updates
  public static final int kLongCANTimeoutMs = 30; //use for constructors

  public static double kDriveEncoderResolution = 2048.0 / (Math.PI * 2);//the unit of "2"is rad 2_rad

  public static double autoInitAngle = 0.0;
  public static boolean isAutoEnd = false;
  public static double driveTrainAngle = 0.0;
  public static double swerveAngle = 0.0;
  public static double swerveSpeed = 0.0;
  public static boolean isShoot = false;
  public static boolean isMove = false;
  public static boolean canClimbUp = false;
  public static double mRobotPitch = 0.0;  
  public static double direc = 0.0;
  public static double minRoll = 100.0;
  public static double maxRoll = 0.0;
  public static double lastValue = 0.0;
  public static double lastDirec = 0.0;

  public static final double driveAngleDeadband = 0.01;
  public static final double stickDeadband = 0.1;
  public static double mDriveSpeedKp = 1.0;
  public static final double mSteerSpeedKp = 1.0;
  
  public static double limitAToB(double value, Double min, double max) {
    if (value > max) {
      return max;
    }
    if (value < min) {
      return min;
    }
    return value;
  }

  public static final class CommonConstants {
    public static final double mSwerveAngleFX_VoltageCompSaturation = 10.0;
    public static final double mSwerveDriveFX_VoltageCompSaturation = 12.0;
  }

  public static final class AutoConstants {
      public static final double kMaxSpeedMetersPerSecond = 3.6;
      // public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
      public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
      public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
  
      public static final double kPXController = 1;
      public static final double kPYController = 1;
      public static final double kPThetaController = 1;
  
      // Constraint for the motion profilied robot angle controller
      public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
          new TrapezoidProfile.Constraints(
              kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

  //upgrad swerve constants
  public static final class TeleopConstants {
    /* Swerve Profiling Values */
    public static final double maxSpeedMetersPerSecond = 4.5;
    public static final double maxAngularVelocity = CommonConstants.mSwerveAngleFX_VoltageCompSaturation;
    public static final double mSwerveDrivePeakValue = 1.0;
    public static final double mSwerveAnglePeakValue = 1.0;

    public static final int pigeonID = 60;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    //TODO: need to be adjusted for your robot!
    public static final double trackWidth = Units.inchesToMeters(22.4);//Robot drive train width(left to right)
    public static final double wheelBase = Units.inchesToMeters(22.4);//Robot drive train length(front to back)
    public static final double wheelDiameter = Units.inchesToMeters(3.94);//Robot drive wheel diameter
    public static final double wheelCircumference = wheelDiameter * Math.PI;//Robot drive wheel circumference

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    //TODO: need to be adjusted for your robot!
    public static final double driveGearRatio = 6.12; //L3
    public static final double angleGearRatio = 150 / 7;//mk4i

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit =  25;//25
    public static final int anglePeakCurrentLimit = 40;//40
    public static final double anglePeakCurrentDuration = 0.5;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;//35
    public static final int drivePeakCurrentLimit = 60;//65
    public static final double drivePeakCurrentDuration = 0.5;
    public static final boolean driveEnableCurrentLimit = true;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.6;//0.6
    public static final double angleKI = 0.0;
    public static final double angleKD = 12.0;
    public static final double angleKV = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.10;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKV = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
    public static final double driveVelocity = (2.44 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /* Motor Inverts */
    // public static final TalonFXInvertType driveMotorInvert = TalonFXInvertType.CounterClockwise;//true
    // public static final TalonFXInvertType angleMotorInvert = TalonFXInvertType.Clockwise;//false can be addjust
    public static final boolean driveMotorInvert = false;//true
    public static final boolean angleMotorInvert = false;//false can be addjust

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;//true

    //TODO: need to be adjusted for your robot!
    public static final double[] kModAngleOffset = {
      0.0 + 7.1 - 90.0 -18.0 -20.0,  //Front Left 181.23,
      0.0 + 55.64,   //Front Right 357.0,//180.25
      0.0 - 5.6 + 180.0,    //Back Left -14.5,  
      0.0 + 28.1 + 180.0    //Back Right 254.44
    };

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
        public static final int driveMotorID = 7;
        public static final int angleMotorID = 8;
        public static final int canCoderID = 2;
        public static final double angleOffset = kModAngleOffset[0];
        public static final SwerveModConstants constants = 
            new SwerveModConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
        public static final int driveMotorID = 9;
        public static final int angleMotorID = 10;
        public static final int canCoderID = 3;
        public static final double angleOffset = kModAngleOffset[1];
        public static final SwerveModConstants constants = 
            new SwerveModConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
    
    /* Back Left Module - Module 2 */
    public static final class Mod2 {
        public static final int driveMotorID = 5;
        public static final int angleMotorID = 6;
        public static final int canCoderID = 1;
        public static final double angleOffset = kModAngleOffset[2];
        public static final SwerveModConstants constants = 
            new SwerveModConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */ //moldule 3 missalined 10/19/21 at 7:40
    public static final class Mod3 {
        public static final int driveMotorID = 11;
        public static final int angleMotorID = 12;
        public static final int canCoderID = 4;
        public static final double angleOffset = kModAngleOffset[3];
        public static final SwerveModConstants constants = 
            new SwerveModConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

}
}
