// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

/** Add your docs here. */
public class Conversions {
    
    /**
     * @param old old Position Counts
     * @return new Position Counts
     */
    public static double positionOldToNew(double old) {
        double New = old / 2048;
        return New;
    }

    /**
     * @param New new Postition Counts
     * @return old Position Counts
     */
    public static double positionNewToOld(double New) {
        double old = New * 2048;
        return old;
    }

    /**
     * @param old old Velocity Counts
     * @return new Velocity Counts
     */
    public static double velocityOldToNew(double old) {
        double New = old * 10 / 2048;
        return New;
    }

    /**
     * @param New new Velocity Counts
     * @return old Velocity Counts
     */
    public static double velocityNewToOld(double New) {
        double old = New * 2048 / 10;
        return old;
    }

    /**
     * @param counts Falcon Position Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double falconToDegrees(double counts, double gearRatio) {
        double oldCounts = positionNewToOld(counts);
        return oldCounts * (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Position Counts
     */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        double ticks = degrees / (360.0 / (gearRatio * 2048.0));
        double counts = positionOldToNew(ticks);
        return counts;
    }

    /**
     * @param counts Falcon Position Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @param circumference Wheel circumference
     * @return meters counts of the robot
     */
    public static double falconToMeters(double counts, double gearRatio, double circumference) {
        double degrees = falconToDegrees(counts, gearRatio);
        double meters = degrees * 360 / circumference;
        return meters;
    }

    /**
     * @param velocityCounts Falcon Velocity Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double oldVelocityCounts = velocityNewToOld(velocityCounts);
        double motorRPM = oldVelocityCounts * (600.0 / 2048.0);        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param RPM RPM of mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        double newSensorCounts = velocityOldToNew(sensorCounts);
        return newSensorCounts;
    }

    /**
     * @param velocitycounts Falcon Velocity Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Velocity MPS
     */
    public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
        double oldVelocityCounts = velocityNewToOld(velocitycounts);
        double wheelRPM = falconToRPM(oldVelocityCounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double MPSToFalcon(double velocity, double circumference, double gearRatio){
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        double newWheelVelocity = velocityOldToNew(wheelVelocity);
        return newWheelVelocity;
    }
}
