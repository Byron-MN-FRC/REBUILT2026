package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.Turret;

/**
 * AimAtTarget uses the turret-mounted Limelight to detect AprilTags around the
 * hub, computes the offset target point from visible tags, and rotates the
 * turret to aim at that point using a closed-loop visual servo approach.
 * <p>
 * The target point is defined by offsets from specific AprilTags:
 * <pre>
 * Tag  X Offset (cm)   Y Offset (cm, depth)
 *  2    +58 (right)      0
 * 11    +58 (right)     +36
 * 10    +36 (right)     +56
 *  9      0             +56
 *  8    -58 (left)      +36
 *  5    -58 (left)       0
 * </pre>
 * The command uses up to the two lowest-ambiguity tags to average the computed
 * target position for better accuracy.
 * <p>
 * Distance is calculated on the ground plane (ignoring vertical/height).
 * The turret is rotated to drive the horizontal angle to the target toward zero.
 */
public class AimAtTarget extends Command {

    private static final String LL_NAME = Constants.VisionConstants.TURRET_CAM;

    // Valid tag IDs and their offsets in METERS (converted from cm)
    // Offsets are relative to the tag position in camera space:
    //   X offset: positive = right, negative = left
    // Valid tag IDs and their offsets in METERS (converted from cm)
    // Offsets are relative to each tag's position, converted to camera frame:
    //   X offset: tag's right = camera's left (negated), tag's left = camera's right
    //   Z offset: "behind" the tag = further from camera = positive Z
    // (In Limelight camera space: X = right, Y = down, Z = forward)
    private static final int[] VALID_TAG_IDS = { 2, 5, 8, 9, 10, 11 };
    private static final double[][] TAG_OFFSETS = {
        // { tagID, xOffsetMeters (camera frame), zOffsetMeters (behind tag) }
        {  2,  0.00,  0.58 },  // 0 lateral,        58cm behind
        { 11, -0.36,  0.58 },  // 36cm right of tag -> camera left, 58cm behind
        { 10,  0.00,  0.58 },  // 0 lateral,        58cm behind
        {  9, -0.36,  0.58 },  // 36cm right of tag -> camera left, 58cm behind
        {  8,  0.36,  0.58 },  // 36cm left of tag  -> camera right, 58cm behind
        {  5,  0.00,  0.58 },  // 0 lateral,        58cm behind
    };

    private final Turret m_turret;

    // SmartDashboard key prefix — Elastic will see these under "SmartDashboard/Targeting/..."
    private static final String SD_PREFIX = "Targeting/";

    // Distance-to-RPM lookup table with linear interpolation.
    // Key = distance to target in METERS, Value = shooter RPM.
    // Seed values converted from existing DistanceVelocityMap (feet→meters).
    // UPDATE THESE WITH REAL TESTING DATA.
    private static final TreeMap<Double, Double> DISTANCE_RPM_MAP = new TreeMap<>() {{
        put(0.00, 2000.0);   // minimum distance — low RPM
        put(2.39, 2000.0);   // ~7'10" — close
        put(3.05, 2125.0);   // ~10'   — close/climb position
        put(3.53, 2125.0);   // ~11'7"
        put(4.17, 2400.0);   // ~13'8" — middle
        put(5.43, 2600.0);   // ~17'10" — far
    }};

    // Low-pass filter smoothing factor (0.0 = ignore new data, 1.0 = no filtering)
    // Lower values = smoother but slower response. 0.15-0.25 is a good range.
    private static final double FILTER_ALPHA = 0.20;

    // Proportional gain for the visual servo. Controls how aggressively the turret
    // corrects toward the target. 1.0 = full correction each frame (causes overshoot),
    // 0.3-0.5 = gentle correction that converges without oscillation.
    private static final double AIM_GAIN = 0.35;

    // Fixed lateral offset applied AFTER tag-based target computation.
    // Compensates for the camera lens being offset from the turret rotation axis.
    // Positive = shift aim right, negative = shift aim left.  Units: meters.
    // Tunable via SmartDashboard "Targeting/AIM_OFFSET_X"
    private static final double AIM_OFFSET_X_DEFAULT = -1.0;

    private double filteredAngle = 0.0;
    private double filteredDistance = 0.0;
    private boolean filterInitialized = false;

    public AimAtTarget(Turret turret) {
        m_turret = turret;
        addRequirements(turret);
        // Publish immediately at construction so we can verify the code is loaded
        SmartDashboard.putBoolean(SD_PREFIX + "Command Active", false);
        SmartDashboard.putString(SD_PREFIX + "Debug", "Constructed");
        // Publish tunable offset — setDefaultNumber won't overwrite a value
        // you've already changed on the dashboard (unlike putNumber which always overwrites)
        SmartDashboard.setDefaultNumber("AIM_OFFSET_X", AIM_OFFSET_X_DEFAULT);
    }

    @Override
    public void initialize() {
        m_turret.isActive = true;
        filteredAngle = 0.0;
        filteredDistance = 0.0;
        filterInitialized = false;
        SmartDashboard.putBoolean(SD_PREFIX + "Command Active", true);
        SmartDashboard.putString(SD_PREFIX + "Debug", "Initialized");
    }

    @Override
    public void execute() {
        // Use NetworkTables-based getRawFiducials (more reliable than JSON getLatestResults)
        RawFiducial[] allFiducials = LimelightHelpers.getRawFiducials(LL_NAME);
        boolean hasTarget = LimelightHelpers.getTV(LL_NAME);

        if (allFiducials == null || allFiducials.length == 0 || !hasTarget) {
            // No targets — hold current turret position (don't move)
            updateDashboard("None", filteredAngle, filteredDistance, false, 0, 0);
            SmartDashboard.putString(SD_PREFIX + "Debug",
                "No fiducials | hasTarget: " + hasTarget
                + " | holding position");
            return;
        }

        // Filter to only our valid tag set — use ALL valid tags for stability
        List<RawFiducial> validTags = new ArrayList<>();
        StringBuilder allSeenIds = new StringBuilder();
        for (RawFiducial fid : allFiducials) {
            allSeenIds.append(fid.id).append(" ");
            if (isValidTag(fid.id)) {
                validTags.add(fid);
            }
        }

        if (validTags.isEmpty()) {
            // Valid tags not in view — hold current turret position
            updateDashboard("None", filteredAngle, filteredDistance, false, 0, 0);
            SmartDashboard.putString(SD_PREFIX + "Debug",
                "No valid tags in set | Saw IDs: " + allSeenIds.toString().trim());
            return;
        }

        // Use ALL valid tags — they should all compute the same target point,
        // so averaging more tags gives a more stable result
        int tagsToUse = validTags.size();

        // For each tag, reconstruct camera-space position from
        // txnc (horizontal angle in degrees) and distToCamera (meters).
        // Then apply the offset to find the target point.
        double sumTargetX = 0.0;
        double sumTargetZ = 0.0;
        StringBuilder tagIdStr = new StringBuilder();

        for (int i = 0; i < tagsToUse; i++) {
            RawFiducial fid = validTags.get(i);
            double txRad = Math.toRadians(fid.txnc);
            double tagX = fid.distToCamera * Math.sin(txRad);
            double tagZ = fid.distToCamera * Math.cos(txRad);

            // Get the offset for this tag
            double[] offset = getOffsetForTag(fid.id);
            double xOffset = offset[0];
            double zOffset = offset[1];

            // Compute target point in camera space
            double targetX = tagX + xOffset;
            double targetZ = tagZ + zOffset;

            sumTargetX += targetX;
            sumTargetZ += targetZ;

            if (i > 0) tagIdStr.append(", ");
            tagIdStr.append(fid.id);
        }

        // Average the target positions from all valid tags
        double avgTargetX = sumTargetX / tagsToUse;
        double avgTargetZ = sumTargetZ / tagsToUse;

        // Apply fixed lateral aim offset (tunable via dashboard)
        double aimOffsetX = SmartDashboard.getNumber("AIM_OFFSET_X", -.5);
        avgTargetX += aimOffsetX;

        // Compute raw horizontal angle and ground-plane distance
        double rawAngleDeg = Math.toDegrees(Math.atan2(avgTargetX, avgTargetZ));
        double rawDistance = Math.sqrt(avgTargetX * avgTargetX + avgTargetZ * avgTargetZ);

        // Apply exponential moving average filter to smooth output
        if (!filterInitialized) {
            filteredAngle = rawAngleDeg;
            filteredDistance = rawDistance;
            filterInitialized = true;
        } else {
            filteredAngle = FILTER_ALPHA * rawAngleDeg + (1.0 - FILTER_ALPHA) * filteredAngle;
            filteredDistance = FILTER_ALPHA * rawDistance + (1.0 - FILTER_ALPHA) * filteredDistance;
        }

        // Visual servo: rotate the turret to drive the filtered angle toward zero.
        // Apply AIM_GAIN so we only correct a fraction of the error each cycle,
        // preventing overshoot and oscillation.
        double currentTurretAngle = m_turret.getAngleDegrees();
        double correction = filteredAngle * AIM_GAIN;
        double newTurretAngle = currentTurretAngle + correction;
        m_turret.aimDegrees(newTurretAngle);

        // Compute target RPM from interpolating lookup table
        double targetRPM = getTargetRPM(filteredDistance);

        // Update dashboard
        updateDashboard(tagIdStr.toString(), filteredAngle, filteredDistance, true, tagsToUse, targetRPM);
        SmartDashboard.putNumber(SD_PREFIX + "Raw Angle (deg)", rawAngleDeg);
        SmartDashboard.putString(SD_PREFIX + "Debug",
            "Aiming | tags: " + tagIdStr + " | raw: "
            + String.format("%.1f", rawAngleDeg) + " | filtered: "
            + String.format("%.1f", filteredAngle) + " | dist: "
            + String.format("%.2f", filteredDistance)
            + " | rpm: " + String.format("%.0f", targetRPM)
            + " | offsetX: " + String.format("%.3f", aimOffsetX));
    }

    @Override
    public void end(boolean interrupted) {
        // Hold the current turret position — don't snap back to 0
        m_turret.isActive = false;
        updateDashboard("None", 0.0, 0.0, false, 0, 0);
        SmartDashboard.putBoolean(SD_PREFIX + "Command Active", false);
        SmartDashboard.putString(SD_PREFIX + "Debug", interrupted ? "Interrupted" : "Ended");
    }

    @Override
    public boolean isFinished() {
        return false; // Runs while button is held
    }

    // --- Helper methods ---

    /**
     * Checks if the given tag ID is in our valid set.
     */
    private boolean isValidTag(int tagId) {
        for (int id : VALID_TAG_IDS) {
            if (id == tagId) return true;
        }
        return false;
    }

    /**
     * Returns the [xOffset, zOffset] in meters for a given tag ID.
     * Returns [0, 0] if the tag is not in the offset table (shouldn't happen
     * if isValidTag was checked first).
     */
    private double[] getOffsetForTag(int tagId) {
        for (double[] entry : TAG_OFFSETS) {
            if ((int) entry[0] == tagId) {
                return new double[] { entry[1], entry[2] };
            }
        }
        return new double[] { 0.0, 0.0 };
    }

    /**
     * Linearly interpolates the required shooter RPM for a given distance (meters)
     * using the DISTANCE_RPM_MAP lookup table.
     * <p>
     * - Below the smallest distance: returns the lowest RPM in the table.
     * - Above the largest distance: returns the highest RPM in the table.
     * - Between two entries: linearly interpolates.
     */
    private static double getTargetRPM(double distanceMeters) {
        // At or below minimum distance
        Map.Entry<Double, Double> floor = DISTANCE_RPM_MAP.floorEntry(distanceMeters);
        Map.Entry<Double, Double> ceil = DISTANCE_RPM_MAP.ceilingEntry(distanceMeters);

        if (floor == null) {
            // Distance is below all entries — use the lowest RPM
            return DISTANCE_RPM_MAP.firstEntry().getValue();
        }
        if (ceil == null) {
            // Distance is above all entries — use the highest RPM
            return DISTANCE_RPM_MAP.lastEntry().getValue();
        }
        if (floor.getKey().equals(ceil.getKey())) {
            // Exact match
            return floor.getValue();
        }

        // Linear interpolation between floor and ceiling entries
        double dLow = floor.getKey();
        double dHigh = ceil.getKey();
        double rpmLow = floor.getValue();
        double rpmHigh = ceil.getValue();
        double t = (distanceMeters - dLow) / (dHigh - dLow);
        return rpmLow + t * (rpmHigh - rpmLow);
    }

    /**
     * Updates all SmartDashboard entries under the "Targeting/" prefix.
     * In Elastic, these appear under SmartDashboard/Targeting/ and can be
     * added to a custom Elastic tab.
     */
    private void updateDashboard(String tagsInUse, double angle, double distance,
                                  boolean visible, int tagCount, double targetRPM) {
        SmartDashboard.putString(SD_PREFIX + "Tags In Use", tagsInUse);
        SmartDashboard.putNumber(SD_PREFIX + "Angle to Target (deg)", angle);
        SmartDashboard.putNumber(SD_PREFIX + "Distance to Target (m)", distance);
        SmartDashboard.putBoolean(SD_PREFIX + "Target Visible", visible);
        SmartDashboard.putNumber(SD_PREFIX + "Tags Seen", tagCount);
        SmartDashboard.putNumber(SD_PREFIX + "Target RPM", targetRPM);
    }
}
