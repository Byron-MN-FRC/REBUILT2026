// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class TurretCam {
    /*
     * Returns the horizontal angle error from the Limelight.
     * Negative value indicates target is to the left (robot needs to turn counterclockwise to center).
     * Positive value indicates target is to the right (robot needs to turn clockwise to center).
     */
    public static double getAngleError() {
        if (targetLocated()) {
            return -LimelightHelpers.getTX(Constants.VisionConstants.TURRET_CAM);
        } else {
            return 0;
        }
    }

    public static boolean targetLocated() {
        return LimelightHelpers.getTV(Constants.VisionConstants.TURRET_CAM);
    } 

    public static boolean targetLocked() {
        return targetLocated() && Math.abs(getAngleError()) < Constants.VisionConstants.ANGLE_ERROR_THRESHOLD;
    }

    public static double getDistance() {
        if (LimelightHelpers.getTV(Constants.VisionConstants.TURRET_CAM)) {
            var robotPos = LimelightHelpers.getBotPose2d(Constants.VisionConstants.TURRET_CAM).getTranslation();
            var alliance = Constants.DriveConstants.MyAlliance();
            
            if (alliance == Alliance.Blue) {
                return robotPos.getDistance(Constants.FieldConstants.BLUE_HUB_CENTER);
            } else {
                return robotPos.getDistance(Constants.FieldConstants.RED_HUB_CENTER);
            }
        }
        return 0;
    }
}