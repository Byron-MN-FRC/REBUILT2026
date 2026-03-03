package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;

public class Vision extends SubsystemBase {

    // TODO fix
    public boolean tempDisable = false;
    public double timestampToReEnable;

    private Pose2d autoStartPose = new Pose2d();
    public int lastAlignmentTarget = 1;

    public Vision() {

        //this is where the camera is relative to the center of the robot
        LimelightHelpers.setCameraPose_RobotSpace(
                Constants.VisionConstants.LIMELIGHT_NAME,
                Constants.VisionConstants.CAMERA_POSE_ROBOT_SPACE[0],
                Constants.VisionConstants.CAMERA_POSE_ROBOT_SPACE[1],
                Constants.VisionConstants.CAMERA_POSE_ROBOT_SPACE[2],
                Constants.VisionConstants.CAMERA_POSE_ROBOT_SPACE[3],
                Constants.VisionConstants.CAMERA_POSE_ROBOT_SPACE[4],
                Constants.VisionConstants.CAMERA_POSE_ROBOT_SPACE[5]
        );

        LimelightHelpers.SetFiducialIDFiltersOverride(
                Constants.VisionConstants.LIMELIGHT_NAME, 
                Constants.FieldConstants.TAGS_FOR_POSE_ESTIMATION);
    }

    @Override
    public void periodic() {
        scanForAlignmentTargets(Constants.VisionConstants.LIMELIGHT_NAME);
        updatePoseEstimator(Constants.VisionConstants.LIMELIGHT_NAME);

        // TODO fix
        if (timestampToReEnable < Utils.getCurrentTimeSeconds() && tempDisable == true) {
            tempDisable = false;
        }

        if (DriverStation.isAutonomous() && !DriverStation.isEnabled()) {
            checkAutoStartPose();
        }

    }

    public Alliance MyAlliance() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            return ally.get() == Alliance.Red ? Alliance.Red : Alliance.Blue;
        } else {
            return null;
        }
    }

    /**
     * If the Limelight reports a valid target and its tag ID matches any
     * in {@code Constants.VisionConstants.TAGS_FOR_AUTO_ALIGNMENT}, update {@link #lastAlignmentTarget}.
     * Otherwise do nothing.
     * <p>
     * This is intented to be called periodically and used when only a certain
     * subset of April Tags are intended to be valid alignment spots.
     * Using only this small subset of targets defined in {@code TAGS_FOR_AUTO_ALIGNMENT}
     * increases the likelihood that the robot will align to the correct target.
     *
     * @param llName Limelight camera name
     */
    public void scanForAlignmentTargets(String llName) {
        if (LimelightHelpers.getTV(llName)) {
            for (int fidID : Constants.FieldConstants.TAGS_FOR_AUTO_ALIGNMENT) {
                if (LimelightHelpers.getFiducialID(llName) == fidID) {
                    lastAlignmentTarget = fidID;
                    return;
                }
            }
        }
    }

    /**
     * Temporarily disables the addVisionMeasurements method in Robot.java
     * TODO fix
     * The purpose of this method is to remove errors caused during resetting
     * the rotation of the robot when the cameras can see an April Tag
     * 
     * @Param seconds The time period to disable for (tested at .5 seconds)
     * @return void
     */
    public void tempDisable(double seconds) {
        tempDisable = true;
        double currentTime = Utils.getCurrentTimeSeconds();
        timestampToReEnable = currentTime + seconds;
    }

    public void updateAutoStartPosition(String autoName) {

        // Instant Command is the name of the "None" Auto

        if (!autoName.equals("InstantCommand")) {
            try {
                autoStartPose = PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).getStartingDifferentialPose();
            } catch (Exception e) {
                System.out.println(e.getMessage());
                autoStartPose = new Pose2d();
            }
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                autoStartPose = FlippingUtil.flipFieldPose(autoStartPose);
            }
        } else {
            autoStartPose = new Pose2d();
        }

    }

    public void checkAutoStartPose() {
        // For auto set-up
        if (!autoStartPose.equals(new Pose2d())) {
            Translation2d currentT2D = Robot.getInstance().drivetrain.getState().Pose.getTranslation();
            double distance = autoStartPose.getTranslation().getDistance(currentT2D);
            // difference between the goal angle and current angle arccos(cos(a-b))
            double rot_distance = Math.acos(autoStartPose.getRotation().getCos() *
                    Robot.getInstance().drivetrain.getState().Pose.getRotation().getCos() +
                    autoStartPose.getRotation().getSin() *
                            Robot.getInstance().drivetrain.getState().Pose.getRotation().getSin());

            // SmartDashboard.putNumber("Auto config distance", distance);
            // SmartDashboard.putNumber("Auto config rotation distance", rot_distance);
            if (distance < 0.2 && (Units.radiansToDegrees(rot_distance) < 4)) {

                LimelightHelpers.setLEDMode_ForceOn(Constants.VisionConstants.LIMELIGHT_NAME);
            } else {
                LimelightHelpers.setLEDMode_ForceOff(Constants.VisionConstants.LIMELIGHT_NAME);
            }
        } else {
            LimelightHelpers.setLEDMode_ForceOff(Constants.VisionConstants.LIMELIGHT_NAME);
        }
    }

    public void updatePoseEstimator(String llName) {

        /*
         * This example of adding Limelight is very simple and may not be sufficient for
         * on-field use.
         * Users typically need to provide a standard deviation that scales with the
         * distance to target
         * and changes with number of tags available.
         *
         * This example is sufficient to show that vision integration is possible,
         * though exact implementation
         * of how to use vision should be tuned per-robot and to the team's
         * specification.
         */
        var driveState = Robot.getInstance().drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

        LimelightHelpers.SetRobotOrientation(llName, headingDeg, 0, 0, 0, 0, 0);
        var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
        if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 1.5 /* Originally 2.0 */ && !tempDisable) {
            Robot.getInstance().drivetrain.addVisionMeasurement(llMeasurement.pose,llMeasurement.timestampSeconds);
        }
    }
}