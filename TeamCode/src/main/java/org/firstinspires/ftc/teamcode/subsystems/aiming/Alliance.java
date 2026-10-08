package org.firstinspires.ftc.teamcode.subsystems.aiming;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Which alliance the robot is on, and the two HIVE cells that alliance shoots into, as that alliance's driver sees
 * them from their own driver station. The two driver stations face each other, so the red driver (facing +x) has
 * the upper cell on their left, and the blue driver (facing -x) has the lower cell on their left.
 * In the Pedro Pathing visualizer the red hive is on the left half of the field and the blue hive on the right.
 * The robot can't sense its alliance: its coordinates are whatever its autonomous declared as the start, so the
 * alliance is chosen by picking the red or blue autonomous, which saves it (Robot.saveAlliance()) for the TeleOp.
 */
public enum Alliance {
    RED("Red", ShotTarget.RED_UPPER, ShotTarget.RED_LOWER),
    BLUE("Blue", ShotTarget.BLUE_LOWER, ShotTarget.BLUE_UPPER);

    public final String label;           // shown on telemetry
    public final ShotTarget leftCell;    // the cell on this alliance's driver's left (X)
    public final ShotTarget rightCell;   // the cell on this alliance's driver's right (B)

    /**
     * @param label     the name shown on telemetry
     * @param leftCell  the alliance's cell on the driver's left
     * @param rightCell the alliance's cell on the driver's right
     */
    Alliance(String label, ShotTarget leftCell, ShotTarget rightCell) {
        this.label = label;
        this.leftCell = leftCell;
        this.rightCell = rightCell;
    }

    /**
     * Turns a pose written for the red side into the same pose for this alliance. The alliances' starting spots and
     * routines are 180 deg rotations of each other about the field center (not mirror images: a mirror would put
     * blue at the top of the field facing the same way as red, when blue really starts at the bottom facing the
     * other way). So a blue pose is the red one rotated: x -> 2 * centerX - x, y -> 2 * centerY - y, and the heading
     * turned by 180 deg. Lets a routine be written once, with red coordinates, and run on either side.
     *
     * @param redSidePose a pose on the red half, in Pedro field coordinates
     * @return the pose unchanged for RED, rotated 180 deg about the field center for BLUE
     */
    public Pose fromRedSide(Pose redSidePose) {
        if (this == RED) {
            return redSidePose;
        }
        return new Pose(2.0 * RobotConstants.FIELD_SYMMETRY_CENTER_X_INCHES - redSidePose.x(),
                2.0 * RobotConstants.FIELD_SYMMETRY_CENTER_Y_INCHES - redSidePose.y(),
                Angle.normalize(redSidePose.heading() + Math.PI)); // turned around, wrapped into [0, 2 PI)
    }

}
