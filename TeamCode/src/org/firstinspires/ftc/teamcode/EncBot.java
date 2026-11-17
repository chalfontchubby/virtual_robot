package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import virtual_robot.util.AngleUtils;

/**
 * Utility class that represents a robot with mecanum drive wheels and three "dead-wheel" encoders.
 *
 * The dead wheels are two forward-facing wheels, one each side of the robot (left and right),
 * plus one sideways-facing wheel at the centre (X). Comparing the left and right wheels gives
 * the turn; their average gives forward movement; the X wheel gives sideways movement.
 *
 * This isn't an OpMode itself. OpModes such as TestOdom create one and call its methods.
 * Distances are in inches and angles in radians.
 */
public class EncBot {

    // Robot dimensions in inches. Only the encoder values marked "used" below matter to this
    // class; the others are here for reference.
    public final double WHEEL_DIAMETER = 4;
    public final double INTER_WHEEL_WIDTH = 16;
    public final double INTER_WHEEL_LENGTH = 14;
    public final double TICKS_PER_DRIVE_ROTATION = 1120;
    public final double TICKS_PER_ENCODER_ROTATION = 1120;
    public final double ENCODER_WHEEL_DIAMETER = 2;
    // Used: encoder ticks for one turn of a dead wheel.
    private final double ENCODER_TICKS_PER_REVOLUTION = 1120;
    // Used: distance a dead wheel rolls in one turn (pi x its 2 inch diameter).
    private final double ENCODER_WHEEL_CIRCUMFERENCE = Math.PI * 2.0;
    // Used: distance between the left and right dead wheels.
    private final double ENCODER_WIDTH = 12.0;

    public final DcMotorEx[] motors = new DcMotorEx[4]; //back_left, front_left, front_right, back_right
    public final DcMotorEx[] encoders = new DcMotorEx[3]; //right, left, X

    // Encoder readings from the previous updateOdometry() call.
    public int[] prevTicks = new int[3];

    // Current position estimate: {x, y, heading}. Heading 0 faces along +X, and
    // heading increases counter-clockwise.
    public double[] pose = new double[3];

    /**
     * Look up the motors and encoders. Call this once, from the OpMode's init.
     */
    public void init(HardwareMap hwMap){
        String[] motorNames =  new String[]{"back_left_motor", "front_left_motor", "front_right_motor", "back_right_motor"};
        for (int i=0; i<4; i++) motors[i] = hwMap.get(DcMotorEx.class, motorNames[i]);
        // The left motors face the opposite way, so reverse them to make positive power
        // drive every wheel forward.
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        // Dead-wheel encoders are wired into motor ports, so they are read as DcMotorEx objects
        // that never get any power.
        String[] encoderNames = new String[]{"enc_right", "enc_left", "enc_x"};
        for (int i=0; i<3; i++) encoders[i] = hwMap.get(DcMotorEx.class, encoderNames[i]);
    }

    /**
     * Drive the robot.
     * @param px    strafe power, positive = right
     * @param py    forward power, positive = forward
     * @param pa    turn power, positive = left (counter-clockwise)
     */
    public void setDrivePower(double px, double py, double pa){
        // Mecanum mixing: each wheel's power is a sum of the forward, strafe and turn demands.
        double[] p = new double[4];
        p[0] = -px + py - pa;
        p[1] = px + py - pa;
        p[2] = -px + py + pa;
        p[3] = px + py + pa;
        // If any power exceeds 1, scale all four down by the same factor to keep the direction.
        double max = Math.max(1, Math.max(Math.abs(p[0]), Math.max(Math.abs(p[1]), Math.max(Math.abs(p[2]), Math.abs(p[3])))));
        if (max > 1) for (int i=0; i<4; i++) p[i] /= max;
        for (int i=0; i<4; i++) motors[i].setPower(p[i]);
    }

    /**
     * Tell the odometry where the robot is now. Call this before the first updateOdometry().
     */
    public void resetOdometry(double x, double y, double headingRadians){
        pose[0] = x;
        pose[1] = y;
        pose[2] = headingRadians;
        // Remember the current readings, so the next update only counts movement from here.
        for (int i=0; i<3; i++) prevTicks[i] = encoders[i].getCurrentPosition();
    }

    /**
     * Update the position estimate from the movement since the last call, and return it.
     * Call this every loop: the steps need to be small for the maths to stay accurate.
     * @return {x, y, heading in radians}
     */
    public double[] updateOdometry(){
        // 1. How many ticks has each dead wheel turned since last time?
        int[] ticks = new int[3];
        for (int i=0; i<3; i++) ticks[i] = encoders[i].getCurrentPosition();
        int newRightTicks = ticks[0] - prevTicks[0];
        int newLeftTicks = ticks[1] - prevTicks[1];
        int newXTicks = ticks[2] - prevTicks[2];
        prevTicks = ticks;

        // 2. Convert ticks to inches. The left wheel is mounted facing the opposite way to the
        // right one, so its count is negated to make "forward" positive for both. The X wheel
        // is negated so that "to the right" is positive.
        double rightDist = newRightTicks * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;
        double leftDist = -newLeftTicks * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;

        // 3. Movement relative to the robot. Forward is the average of the two side wheels.
        // If the right wheel went further than the left, the robot turned left: the difference
        // divided by the distance between the wheels is the turn, in radians.
        double dyR = 0.5 * (rightDist + leftDist);
        double headingChangeRadians = (rightDist - leftDist) / ENCODER_WIDTH;
        double dxR = -newXTicks * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;

        // 4. Rotate that robot-relative movement onto the field, using the heading halfway
        // through this step (a good approximation while the steps are small).
        double avgHeadingRadians = pose[2] + headingChangeRadians / 2.0;
        double cos = Math.cos(avgHeadingRadians);
        double sin = Math.sin(avgHeadingRadians);
        pose[0] += dxR*sin + dyR*cos;
        pose[1] += -dxR*cos + dyR*sin;

        // 5. Update the heading, kept in the range -pi to +pi.
        pose[2] = AngleUtils.normalizeRadians(pose[2] + headingChangeRadians);
        return pose;
    }

    /**
     * @return the latest position estimate, {x, y, heading in radians}, without updating it
     */
    public double[] getPose(){
        return pose;
    }

}
