package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * TeleOp for a tank-style (differential drive) robot: the wheels on each side are driven
 * together, and the robot turns by running the two sides at different speeds. This suits robots
 * with normal wheels, omni wheels, or a mix (for example traction wheels in the middle and omnis
 * at the corners, so the robot turns easily). Unlike mecanum, it can't strafe sideways.
 *
 * Two ways to drive, switched with the X button:
 *   TANK     left stick Y drives the left side, right stick Y drives the right side
 *   ARCADE   left stick Y drives forward/back, right stick X turns
 *
 * Robot configuration: Two Wheel Bot.
 *
 * On a real robot with four drive motors, look up all four and give both motors on each side
 * the same power (see setDrivePower below). The motor names must match the robot's configuration.
 */
@TeleOp(name = "Tank Drive", group = "Learning")
public class TankDrive extends LinearOpMode {

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private IMU imu;

    // Which way of driving is in use. Starts in tank mode.
    private boolean arcadeMode = false;

    // Whether X was held down last time round the loop. Used to spot the moment it's pressed.
    private boolean xWasPressed = false;

    @Override
    public void runOpMode() {

        // ---- Hardware setup ----
        leftDrive = hardwareMap.get(DcMotor.class, "left_motor");
        rightDrive = hardwareMap.get(DcMotor.class, "right_motor");

        // The left motor faces the opposite way to the right one, so reverse it to make positive
        // power drive both sides forward. If your robot spins when it should drive straight,
        // this is the line to check.
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Stop sharply when the sticks are released, rather than coasting.
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        telemetry.addData("Status", "Ready. Press START.");
        telemetry.addData("Mode", "TANK (press X to switch)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ---- Switch mode when X is pressed ----
            // gamepad1.x is true for as long as the button is held, and the loop runs many times
            // a second. Switching whenever x is true would flip the mode over and over while it's
            // held. Instead, switch only when x is pressed now but wasn't last time round the loop.
            boolean xIsPressed = gamepad1.x;
            if (xIsPressed && !xWasPressed) {
                arcadeMode = !arcadeMode;   // ! means "not": flips true to false and back
            }
            xWasPressed = xIsPressed;

            // ---- Work out the power for each side ----
            double leftPower;
            double rightPower;

            if (arcadeMode) {
                // Arcade: one stick says how fast to go, the other how much to turn.
                // Stick Y is negative when pushed forward, hence the minus sign.
                double drive = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;

                // To turn right, speed up the left side and slow down the right.
                leftPower = drive + turn;
                rightPower = drive - turn;

                // Full forward plus full turn would ask one side for 2.0, more than a motor can
                // give. If either side is over 1, scale both down by the same amount so the robot
                // still follows the curve you asked for.
                double biggest = Math.max(Math.abs(leftPower), Math.abs(rightPower));
                if (biggest > 1.0) {
                    leftPower = leftPower / biggest;
                    rightPower = rightPower / biggest;
                }
            } else {
                // Tank: each stick drives its own side directly.
                leftPower = -gamepad1.left_stick_y;
                rightPower = -gamepad1.right_stick_y;
            }

            setDrivePower(leftPower, rightPower);

            // ---- Telemetry ----
            telemetry.addData("Mode", arcadeMode ? "ARCADE" : "TANK");
            telemetry.addData("Power", "left %.2f  right %.2f", leftPower, rightPower);
            telemetry.addData("Heading", "%.1f degrees",
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Encoders", "left %d  right %d",
                    leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
            telemetry.addData("Press X", "to switch between TANK and ARCADE");
            telemetry.update();
        }
    }

    /**
     * Set the power for each side of the robot, from -1 (full reverse) to 1 (full forward).
     * Keeping this in one method means a four-motor robot only needs changes here.
     */
    private void setDrivePower(double leftPower, double rightPower) {
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        // Four-motor robot: add the other two motors here too, e.g.
        //   leftBackDrive.setPower(leftPower);
        //   rightBackDrive.setPower(rightPower);
    }
}
