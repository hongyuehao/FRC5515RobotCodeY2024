// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

/** Add your docs here. */
public class Conversions {
    /**
     * @param counts Falcon Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double falconToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Counts
     */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        double ticks = degrees / (360.0 / (gearRatio * 2048.0));
        return ticks;
    }
    
    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Rotations
     */
    public static double degreesToRotations(double degrees, double gearRatio) {
        double ticks = degreesToFalcon(degrees, gearRatio);
        double RPS = ticks * 10 / 2048;
        return RPS;
    }

    /**
     * @param rotations Rotations
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees
     */
    public static double rotationsToDegrees(double rotations, double gearRatio) {
        double ticks = rotations * 2048;
        return falconToDegrees(ticks, gearRatio);
    }

    /**
     * @param positionCounts Falcon Position Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Wheel
     * @return Meters
     */
    public static double falconToMeters(double positionCounts, double circumference, double gearRatio) {
        return positionCounts * (circumference / (gearRatio * 2048.0));
    }

    /**
     * @param rotations Rotations
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @param circumference Circumference of Wheel
     * @return Meters
     */
    public static double rotationToMeters(double rotations, double gearRatio, double circumference) {
        double ticks = rotations * 2048;
        return falconToMeters(ticks, circumference, gearRatio);
    }

    /**
     * @param velocityCounts Falcon Velocity Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param RPM RPM of mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * @param velocitycounts Falcon Velocity Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double falconToMPS(double velocitycounts, double circumference, double gearRatio){
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }
    
    /**
     * @param velocity velocity MPS
     * @param circumference Cirumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return velocity RPS
     */
    public static double MPSToRPS(double velocity, double circumference, double gearRatio) {
        double wheelRPM = (velocity * 60) / circumference;
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    /**
     * @param velocity Velocity RPS
     * @param circumference Circumference of wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return velocity MPS
     */
    public static double RPSToMPS(double velocity, double circumference, double gearRatio) {
        double wheelRPM = velocity * 60;
        return RPSToMPS(wheelRPM, circumference, gearRatio);
    }
}
