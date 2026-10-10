package org.firstinspires.ftc.teamcode.subsystems.aiming;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Distance -> shooter RPM lookup, with linear interpolation between rows. Pure Java, no hardware.
 * Distances below the first row or above the last row get that end row's RPM; the table never extrapolates.
 * Whether a distance is close/far enough to shoot from at all is a separate range (inRange()), which can be
 * narrower than the table: the table's rows exist for interpolation, the range is where shots actually work.
 * To add a row, add it to SHOT_TABLE_DISTANCES_INCHES / SHOT_TABLE_RPMS (keep distances ascending).
 *
 * The curve is deliberately U-shaped: close in, the shot needs more speed to clear the front lip of the CELL.
 * Don't smooth it into a line.
 *
 * Raw measurements behind the table, all against the measured cell (ShotTarget.BLUE_LOWER). Positions are in the
 * old TeleOp frame (Pedro minus 8.5 in on x and y): inches, heading in degrees, 0 = +x, CCW positive.
 * RPMs are shooter motor RPM as shown on telemetry (28 ticks/rev). Keep these for future tuning.
 *   x       y      heading  minRPM  maxRPM
 *   73.5    2.88   270.5    2350    2500
 *   44.89   11.6   228.3    2300    2450
 *   26.15   6.34   226.18   2450    2650
 *   38.697  9.1367 224.1    2350    2500
 *   52.21   8.05   245.58   2300    2450
 *   73.5    5.6    270      2450    2500   <- closest workable point (~42 in)
 *   73.5    0      270      2400    2500
 */
public class ShotTable {
    // =====================================================================================
    // CONSTANTS (only this class uses these; shared ones stay in RobotConstants)
    // =====================================================================================
    // Distance (inches from the target) -> shooter motor RPM, linearly interpolated between rows (see ShotTable).
    // Distances must be ascending. The curve is deliberately U-shaped: close in, the shot needs more speed to
    // clear the front lip of the CELL. RPMs are as read from shooter telemetry (SHOOTER_ENCODER_TICKS_PER_REV).
    public static final double[] SHOT_TABLE_DISTANCES_INCHES = {42.0, 45.0, 48.0, 52.0, 62.4};
    // Every row lowered 25 RPM on 10-09 (was 2475, 2400, 2440, 2460, 2525): with the wheel verified at the table
    // speed, Match Auto 1's shots from its start spot (~48 in) still landed high in the cell or missed over.
    public static final double[] SHOT_TABLE_RPMS = {2450.0, 2375.0, 2415.0, 2435.0, 2500.0};

    // A position is only valid to shoot from within this distance range (inclusive). The 42 in table row exists
    // for interpolation, but 42 in is the edge of what works, so the range starts at 43.
    public static final double SHOT_MIN_DISTANCE_INCHES = 43.0;
    public static final double SHOT_MAX_DISTANCE_INCHES = 62.4;

    private final double[] distancesInches; // ascending
    private final double[] rpms;            // rpms[i] is the shot speed at distancesInches[i]
    private final double minInRangeInches;
    private final double maxInRangeInches;

    /**
     * Builds a table from matching distance and RPM rows.
     *
     * @param distancesInches  row distances, strictly ascending, at least two rows
     * @param rpms             the shooter RPM for each row
     * @param minInRangeInches the closest distance that counts as a valid shot
     * @param maxInRangeInches the farthest distance that counts as a valid shot
     */
    public ShotTable(double[] distancesInches, double[] rpms, double minInRangeInches, double maxInRangeInches) {
        if (distancesInches.length != rpms.length || distancesInches.length < 2) {
            throw new IllegalArgumentException("Shot table needs at least two rows, with one RPM per distance");
        }
        for (int rowIndex = 1; rowIndex < distancesInches.length; rowIndex++) {
            if (distancesInches[rowIndex] <= distancesInches[rowIndex - 1]) {
                throw new IllegalArgumentException("Shot table distances must be strictly ascending");
            }
        }
        // Copies, so later changes to the caller's arrays can't change this table
        this.distancesInches = distancesInches.clone();
        this.rpms = rpms.clone();
        this.minInRangeInches = minInRangeInches;
        this.maxInRangeInches = maxInRangeInches;
    }

    /**
     * Builds the table from the tuned rows and range at the top of this class.
     *
     * @return the robot's shot table
     */
    public static ShotTable fromTunedRows() {
        return new ShotTable(
                SHOT_TABLE_DISTANCES_INCHES,
                SHOT_TABLE_RPMS,
                SHOT_MIN_DISTANCE_INCHES,
                SHOT_MAX_DISTANCE_INCHES);
    }

    /**
     * The shooter RPM for a distance, linearly interpolated between the two surrounding rows.
     * Past either end of the table, returns that end's RPM (no extrapolation).
     *
     * @param distanceInches distance from the robot to the target
     * @return the shooter target RPM
     */
    public double rpmAt(double distanceInches) {
        int lastRowIndex = distancesInches.length - 1;
        if (distanceInches <= distancesInches[0]) {
            return rpms[0];
        }
        if (distanceInches >= distancesInches[lastRowIndex]) {
            return rpms[lastRowIndex];
        }
        // Find the first row at or past the distance; the distance lies between that row and the one before it
        int upperRowIndex = 1;
        while (distancesInches[upperRowIndex] < distanceInches) {
            upperRowIndex++;
        }
        int lowerRowIndex = upperRowIndex - 1;
        // How far (0 to 1) the distance is from the lower row to the upper row
        double fractionBetweenRows = (distanceInches - distancesInches[lowerRowIndex])
                / (distancesInches[upperRowIndex] - distancesInches[lowerRowIndex]);
        return rpms[lowerRowIndex] + fractionBetweenRows * (rpms[upperRowIndex] - rpms[lowerRowIndex]);
    }

    /**
     * Tells whether a distance is within the range shots work from (inclusive at both ends).
     *
     * @param distanceInches distance from the robot to the target
     * @return true if the distance is between the min and max in-range distances
     */
    public boolean inRange(double distanceInches) {
        return distanceInches >= minInRangeInches && distanceInches <= maxInRangeInches;
    }

    /**
     * The closest distance that counts as in range.
     *
     * @return the minimum in-range distance, in inches
     */
    public double getMinInRangeInches() {
        return minInRangeInches;
    }

    /**
     * The farthest distance that counts as in range.
     *
     * @return the maximum in-range distance, in inches
     */
    public double getMaxInRangeInches() {
        return maxInRangeInches;
    }
}
