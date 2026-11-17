package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.*;

/**
 * Example TeleOp for an X-drive robot (four omni wheels mounted at 45 degrees in the corners).
 * Demonstrates driving with the gamepad, a continuous-rotation servo, and reading the IMU,
 * color sensor, distance sensors and drive motor encoders, with the results shown in telemetry.
 *
 * Robot configuration: XDrive Bot.
 *
 * Controls (gamepad1):
 *   left stick      drive forward/back and strafe left/right
 *   right stick X   turn
 *   right trigger   spin the back CR servo (how hard you press sets its speed)
 */
@TeleOp(name = "XDriveBot demo", group = "XBot")
public class XDriveBotDemo extends LinearOpMode {

    DcMotor m1, m2, m3, m4;
    CRServo backServo;
    IMU imu = null;

    public void runOpMode(){
        // ---- Init: everything up to waitForStart() runs when INIT is pressed ----

        // Device names must match those in the robot configuration.
        m1 = hardwareMap.dcMotor.get("back_left_motor");
        m2 = hardwareMap.dcMotor.get("front_left_motor");
        m3 = hardwareMap.dcMotor.get("front_right_motor");
        m4 = hardwareMap.dcMotor.get("back_right_motor");

        // The left motors are mounted facing the opposite way to the right ones, so reverse them
        // to make positive power drive every wheel forward.
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);

        // Zero the encoders, then use RUN_USING_ENCODER so setPower() sets a speed
        // (a fraction of maximum) that the motor controller holds using the encoder.
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        // Tell the IMU how the hub is mounted on the robot (logo up, USB port facing forward)
        // so it can report the robot's yaw/pitch/roll rather than the hub's.
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        ElapsedTime et = new ElapsedTime();
        // Add brief delay for IMU initialization to complete
        while (et.milliseconds() < 150 && !Thread.currentThread().isInterrupted()) continue;

        // A CRServo (continuous-rotation servo) takes a power, like a motor, not a position.
        backServo = hardwareMap.crservo.get("back_crservo");
        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        DistanceSensor backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");

        ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");
        telemetry.addData("Press Start When Ready","");
        telemetry.update();

        // Wait here until START is pressed.
        waitForStart();

        // ---- Run: loop until STOP is pressed ----
        while (opModeIsActive()){

            // px = strafe right, py = forward, pa = turn left (counter-clockwise).
            // Stick y is negative when pushed forward, and pushing the right stick right should
            // turn right, hence the minus signs. Each input has a small dead band so a stick
            // that doesn't quite centre doesn't make the robot creep.
            double px = gamepad1.left_stick_x;
            if (Math.abs(px) < 0.05) px = 0;
            double py = -gamepad1.left_stick_y;
            if (Math.abs(py) < 0.05) py = 0;
            double pa = -gamepad1.right_stick_x;
            if (Math.abs(pa) < 0.05) pa = 0;

            // X-drive mixing: with the wheels at 45 degrees this works out the same as mecanum.
            // Each wheel's power is a sum of the forward, strafe and turn demands.
            double p1 = -px + py - pa;
            double p2 = px + py + -pa;
            double p3 = -px + py + pa;
            double p4 = px + py + pa;

            // The sums can exceed 1. If any does, scale all four down by the same factor so
            // the robot still moves in the requested direction instead of clipping one wheel.
            double max = Math.max(1.0, Math.abs(p1));
            max = Math.max(max, Math.abs(p2));
            max = Math.max(max, Math.abs(p3));
            max = Math.max(max, Math.abs(p4));
            p1 /= max;
            p2 /= max;
            p3 /= max;
            p4 /= max;
            m1.setPower(p1);
            m2.setPower(p2);
            m3.setPower(p3);
            m4.setPower(p4);

            double psrv = gamepad1.right_trigger;
            backServo.setPower(psrv);


            telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("Heading (IMU)", "%.1f degrees", heading);
            telemetry.addData("Front Distance", " %.1f", frontDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Left Distance", " %.1f", leftDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Right Distance", " %.1f", rightDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Back Distance", " %.1f", backDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(),
                    m3.getCurrentPosition(), m4.getCurrentPosition());
            // In a LinearOpMode nothing reaches the screen until telemetry.update() is called.
            telemetry.update();
        }

        // STOP was pressed: make sure the motors are off.
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }
}
