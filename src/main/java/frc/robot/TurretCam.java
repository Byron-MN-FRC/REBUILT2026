// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class TurretCam {

    /*
     * Returns the horizontal angle error from the Limelight.
     * Negative value indicates target is to the left (robot needs to turn counterclockwise to center).
     * Positive value indicates target is to the right (robot needs to turn clockwise to center).
     */
    public static double getAngleError() {
        if (LimelightHelpers.getTV(Constants.VisionConstants.TURRET_CAM)) {
            return -LimelightHelpers.getTX(Constants.VisionConstants.TURRET_CAM);
        } else {
            return 0;
        }
    }

    public static boolean targetLocated() {
        return LimelightHelpers.getTV(Constants.VisionConstants.TURRET_CAM);
    } 
}