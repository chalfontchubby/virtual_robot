package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;

/**
 * Example TeleOp for a mecanum-drive robot. Demonstrates driving with the gamepad, and reading the
 * IMU, color sensor, distance sensors, drive motor encoders and the goBILDA Pinpoint, with the
 * results shown in telemetry.
 *
 * Robot configuration: Mecanum Bot (or MecDynamic Bot).
 *
 * Controls (gamepad1):
 *   left stick      drive forward/back and strafe left/right
 *   left trigger    turn left
 *   right trigger   turn right
 *
 * The maxVel/maxAccel/minAccel telemetry tracks the fastest forward speed and acceleration
 * seen so far, which is handy when tuning RoadRunner or Pedro constraints.
 */
@TeleOp(name = "mecanum bot demo", group = "MecanumBot")
public class MecanumDemo extends LinearOpMode {

    public void runOpMode(){
        // ---- Init: everything up to waitForStart() runs when INIT is pressed ----

        // Device names must match those in the robot configuration.
        DcMotor m1 = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor m2 = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor m3 = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor m4 = hardwareMap.dcMotor.get("back_right_motor");

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

        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        DistanceSensor backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");

        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Downward-facing: reads the color of the field under the centre of the robot.
        ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");
//        OctoQuad octoQuad = hardwareMap.get(OctoQuad.class, "octoquad");

        // Pinpoint odometry computer. Treat wherever the robot is now as (0, 0), heading 0.
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,
                "pinpoint");
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));

        telemetry.addData("Press Start When Ready","");
        telemetry.update();

        // Wait here until START is pressed.
        waitForStart();

        // Previous velocity and time, used to estimate acceleration between loops.
        VectorF oldVel = new VectorF(0,0);
        double oldSecs = getRuntime();
        double maxVel = 0;
        double maxAccel = 0;
        double minAccel = 0;

        // ---- Run: loop until STOP is pressed ----
        while (opModeIsActive()){
            // px = strafe right, py = forward, pa = turn left (counter-clockwise).
            // The stick's y axis is negative when pushed forward, hence the minus sign.
            double px = gamepad1.left_stick_x;
            double py = -gamepad1.left_stick_y;
            double pa = gamepad1.left_trigger - gamepad1.right_trigger;

            // Small dead band so a trigger that isn't quite released doesn't cause creep.
            if (Math.abs(pa) < 0.05) pa = 0;

            // Mecanum mixing: each wheel's power is a sum of the forward, strafe and turn
            // demands. To strafe right, the front-left and back-right wheels go forward while
            // the other two go backward. To turn left, the left side goes backward and the right forward.
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

            // The Pinpoint only refreshes its readings when update() is called, so call it
            // once per loop.
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            double secs = getRuntime();
            VectorF vel = new VectorF((float)pinpoint.getVelX(DistanceUnit.INCH),
                    (float)pinpoint.getVelY(DistanceUnit.INCH));
            // Acceleration = change in velocity / change in time since the previous loop.
            VectorF accel = vel.subtracted(oldVel).multiplied(1.0f/((float)secs - (float)oldSecs));

            // Element 0 is the X component. The Pinpoint reports velocity in the coordinate frame
            // set by setPosition() above (X = the robot's starting forward direction), not
            // relative to the robot, so these are forward-speed figures only while driving
            // straight ahead without turning.
            if (vel.get(0) > maxVel) maxVel = vel.get(0);
            if (accel.get(0) > maxAccel) maxAccel = accel.get(0);
            if (accel.get(0) < minAccel) minAccel = accel.get(0);

            oldVel = vel;
            oldSecs = secs;

            telemetry.addData("maxVel", maxVel);
            telemetry.addData("maxAccel", maxAccel);
            telemetry.addData("minAccel", minAccel);

            telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
//            Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            // With ZYX axis order, firstAngle is rotation about Z, i.e. the robot's heading.
            Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            telemetry.addData("Heading", " %.1f", orientation.firstAngle * 180.0 / Math.PI);
            telemetry.addData("Angular Velocity", "%.1f", imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);
            telemetry.addData("Front Distance", " %.1f", frontDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Left Distance", " %.1f", leftDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Right Distance", " %.1f", rightDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Back Distance", " %.1f", backDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(),
                    m3.getCurrentPosition(), m4.getCurrentPosition());
//            telemetry.addData("Octoquad", "%d %d %d %d", octoQuad.readSinglePosition(0),
//                    octoQuad.readSinglePosition(1), octoQuad.readSinglePosition(2),
//                    octoQuad.readSinglePosition(3));
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
