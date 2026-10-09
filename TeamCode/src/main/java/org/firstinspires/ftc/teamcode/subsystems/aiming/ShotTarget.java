package org.firstinspires.ftc.teamcode.subsystems.aiming;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * One HIVE cell to shoot into: its center point and the direction robots shoot it from. Pure data, no hardware.
 * All coordinates are Pedro field coordinates (inches; heading 0 = +x, counterclockwise positive).
 *
 * Only one cell is measured: the BLUE hive's lower-right cell, as seen in the Pedro Pathing visualizer
 * (the MEASURED_CELL_* constants at the top of this class). Every shot measurement and the whole shot table were
 * taken against that cell.
 * The other three cells are derived from it by the field's symmetry about the point
 * (RobotConstants.FIELD_SYMMETRY_CENTER_X_INCHES, RobotConstants.FIELD_SYMMETRY_CENTER_Y_INCHES), and the shot
 * table carries over to them unchanged because the distances are the same by symmetry.
 *
 * Field layout in the visualizer: the blue hive is on the right, the red hive on the left, and each hive has an
 * upper and a lower cell. Lower cells are shot from below (-y), upper cells from above (+y). The drivers stand on
 * the left edge looking toward +x, so for them "left" is the upper cell and "right" is the lower cell.
 */
public final class ShotTarget {
    // =====================================================================================
    // CONSTANTS (only this class uses these; shared ones stay in RobotConstants)
    // =====================================================================================
    // THE MEASURED CELL: the BLUE hive's LOWER-RIGHT cell, as seen in the Pedro Pathing visualizer. Every shot
    // measurement and the shot table below were taken against this one cell. The other three cells are derived from
    // it by symmetry (see ShotTarget), so this is the only cell position stored.
    // The point is the least-squares intersection of all measured shot headings. It was measured as (72.95, 47.56)
    // in the old TeleOp frame, which is Pedro minus 8.5 in on x and y.
    private static final double MEASURED_CELL_X_INCHES = 81.45;
    private static final double MEASURED_CELL_Y_INCHES = 56.06;
    // Direction from the measured cell to the spot straight in front of it: it's shot from below (-y), i.e. 270 deg
    private static final double MEASURED_CELL_FRONT_DIRECTION_DEGREES = 270.0;

    /** Blue hive, lower cell: the measured cell everything else is derived from. */
    public static final ShotTarget BLUE_LOWER = new ShotTarget("Blue lower",
            MEASURED_CELL_X_INCHES,
            MEASURED_CELL_Y_INCHES,
            MEASURED_CELL_FRONT_DIRECTION_DEGREES);

    /** Blue hive, upper cell: the measured cell flipped over the field's horizontal center line. */
    public static final ShotTarget BLUE_UPPER = BLUE_LOWER.flippedOverHorizontalCenterLine("Blue upper");

    /** Red hive, upper cell: the measured cell rotated 180 deg about the field center. */
    public static final ShotTarget RED_UPPER = BLUE_LOWER.rotated180AboutCenter("Red upper");

    /** Red hive, lower cell: the red upper cell flipped over the field's horizontal center line. */
    public static final ShotTarget RED_LOWER = RED_UPPER.flippedOverHorizontalCenterLine("Red lower");

    public final String name;                   // shown on telemetry
    public final double xInches;                // cell center, Pedro x
    public final double yInches;                // cell center, Pedro y
    // Direction from the cell to the spot straight in front of it, where shots come from (degrees, 0 = +x, CCW
    // positive). The angle limit (RobotConstants.MAX_ANGLE_OFF_CENTER_DEG) is measured either side of this
    public final double frontDirectionDegrees;

    /**
     * @param name                  the cell's name, for telemetry
     * @param xInches               the cell center's Pedro x
     * @param yInches               the cell center's Pedro y
     * @param frontDirectionDegrees direction from the cell to straight in front of it, in degrees
     */
    public ShotTarget(String name, double xInches, double yInches, double frontDirectionDegrees) {
        this.name = name;
        this.xInches = xInches;
        this.yInches = yInches;
        this.frontDirectionDegrees = wrapDegrees(frontDirectionDegrees);
    }

    /**
     * This cell flipped over the horizontal line through the field center (y -> 2 * centerY - y). A direction
     * flips with it: pointing up becomes pointing down, so the angle is negated.
     *
     * @param flippedName the flipped cell's name
     * @return the flipped cell
     */
    public ShotTarget flippedOverHorizontalCenterLine(String flippedName) {
        return new ShotTarget(flippedName,
                xInches,
                2.0 * RobotConstants.FIELD_SYMMETRY_CENTER_Y_INCHES - yInches,
                -frontDirectionDegrees);
    }

    /**
     * This cell rotated 180 deg about the field center (x -> 2 * centerX - x, y -> 2 * centerY - y). A direction
     * rotates with it, so it turns by 180 deg.
     *
     * @param rotatedName the rotated cell's name
     * @return the rotated cell
     */
    public ShotTarget rotated180AboutCenter(String rotatedName) {
        return new ShotTarget(rotatedName,
                2.0 * RobotConstants.FIELD_SYMMETRY_CENTER_X_INCHES - xInches,
                2.0 * RobotConstants.FIELD_SYMMETRY_CENTER_Y_INCHES - yInches,
                frontDirectionDegrees + 180.0);
    }

    /**
     * This cell moved by a fixed amount, for an OpMode whose coordinates are offset from Pedro's (e.g. DriveTest,
     * whose (0, 0) is Pedro (8.5, 8.5)). The front direction doesn't change.
     *
     * @param shiftXInches how far to move the cell in x
     * @param shiftYInches how far to move the cell in y
     * @return the moved cell, with the same name
     */
    public ShotTarget shifted(double shiftXInches, double shiftYInches) {
        return new ShotTarget(name, xInches + shiftXInches, yInches + shiftYInches, frontDirectionDegrees);
    }

    /**
     * Wraps an angle into [0, 360).
     *
     * @param angleDegrees any angle, in degrees
     * @return the same angle wrapped into [0, 360)
     */
    private static double wrapDegrees(double angleDegrees) {
        // Double modulo keeps the result positive for negative angles
        return (angleDegrees % 360.0 + 360.0) % 360.0;
    }

    /**
     * Describes the cell, for telemetry and test messages.
     *
     * @return e.g. "Blue lower (81.45, 56.06)"
     */
    @Override
    public String toString() {
        return String.format(java.util.Locale.US, "%s (%.2f, %.2f)", name, xInches, yInches);
    }
}
