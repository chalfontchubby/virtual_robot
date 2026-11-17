package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.pedropathing.tuning.Constants;

/**
 * Drive the robot around and watch what the goBILDA Pinpoint odometry computer reports: position,
 * velocity and the raw ticks from its two odometry pods.
 *
 * Robot configuration: Mecanum Bot, MecDynamic Bot or XDrive Bot (the robots with a "pinpoint").
 *
 * Controls (gamepad1):
 *   left stick      drive forward/back and strafe left/right
 *   right stick X   turn
 *   A               reset the Pinpoint's position to (0, 0), heading 0
 */
@TeleOp
public class TestPinpoint extends LinearOpMode {

    GoBildaPinpointDriver pinPoint;
    DcMotorEx bl, fl, fr, br;

    public void runOpMode(){

        // ---- Init: everything up to waitForStart() runs when INIT is pressed ----

        // Describe the odometry pods to the Pinpoint: which way each encoder counts, where the
        // pods sit relative to the robot's centre, and how many encoder ticks make an inch
        // (336.88 ticks/inch is goBILDA's swingarm pod).
        // In the simulator these settings only change the raw encoder ticks. The position and
        // velocity come straight from the simulated robot, so they are correct whatever you set.
        pinPoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinPoint.setOffsets(100, 100, DistanceUnit.MM);
        pinPoint.setEncoderResolution(336.88, DistanceUnit.INCH);

        bl = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        fl = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        fr = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        br = hardwareMap.get(DcMotorEx.class, "back_right_motor");
        // RUN_USING_ENCODER: setPower() sets a speed that the motor controller holds.
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // The left motors face the opposite way, so reverse them to make positive power
        // drive every wheel forward.
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        // Show the Pinpoint's starting readings while waiting for START.
        Pose2D pinPose = pinPoint.getPosition();
        double velXInch = pinPoint.getVelX(DistanceUnit.INCH);
        double velYInch = pinPoint.getVelY(DistanceUnit.INCH);
        double headingVelDegrees = pinPoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
        telemetry.addData("Pinpoint Offsets", "xOff: %.1f  yOff: %.1f",
                pinPoint.getXOffset(DistanceUnit.INCH), pinPoint.getYOffset(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Pos", "x: %.1f  y: %.1f  h: %.1f", pinPose.getX(DistanceUnit.INCH),
                pinPose.getY(DistanceUnit.INCH), pinPose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Pinpoint Vel", "x: %.1f  y: %.1f  h: %.1f", velXInch, velYInch, headingVelDegrees);
        telemetry.addData("Pinpoint Encoders", "X: %d  Y: %d", pinPoint.getEncoderX(), pinPoint.getEncoderY());
        telemetry.update();

        // Wait here until START is pressed.
        waitForStart();

        // Not used yet.
        double vxOld = 0;
        long nanosOld = System.nanoTime();

        // ---- Run: loop until STOP is pressed ----
        while (opModeIsActive()){

            // Make wherever the robot is now the new origin.
            if (gamepad1.a){
                pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
            }

            // The Pinpoint only refreshes its readings when update() is called, so call it
            // once per loop, before reading anything.
            pinPoint.update();

            // Two ways of reading the same position: as one Pose2D, or one value at a time.
            // Both are shown below as "Pinpoint Pos" and should agree.
            pinPose = pinPoint.getPosition();
            double pinX = pinPoint.getPosX(DistanceUnit.INCH);
            double pinY = pinPoint.getPosY(DistanceUnit.INCH);
            double pinH = pinPoint.getHeading(AngleUnit.DEGREES);
            // Velocity is in the same coordinate frame as the position, not relative to the robot.
            double pinVX = pinPoint.getVelX(DistanceUnit.INCH);
            double pinVY = pinPoint.getVelY(DistanceUnit.INCH);
            double pinVH = pinPoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

            telemetry.addData("Pinpoint Pos", "x: %.1f  y: %.1f  h: %.1f", pinPose.getX(DistanceUnit.INCH),
                    pinPose.getY(DistanceUnit.INCH), pinPose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Pinpoint Pos", "x: %.1f  y: %.1f  h: %.1f", pinX, pinY, pinH);
            telemetry.addData("Pinpoint Vel", "x: %.1f  y: %.1f  h: %.1f", pinVX, pinVY, pinVH);
            telemetry.addData("Pinpoint Encoders", "X: %d  Y: %d", pinPoint.getEncoderX(), pinPoint.getEncoderY());

            telemetry.update();

            // Drive. Stick y is negative when pushed forward, and pTurn is positive for a turn
            // to the left (counter-clockwise), hence the minus signs.
            double pFwd = -gamepad1.left_stick_y;
            double pRight = gamepad1.left_stick_x;
            double pTurn = -gamepad1.right_stick_x;

            // Mecanum mixing: each wheel's power is a sum of the forward, strafe and turn demands.
            double pBL = pFwd - pRight - pTurn;
            double pFL = pFwd + pRight - pTurn;
            double pFR = pFwd - pRight + pTurn;
            double pBR = pFwd + pRight + pTurn;
            // Meant to scale the powers down if any exceeds 1, but it takes the max of the
            // stick inputs rather than of the wheel powers, so it never scales anything:
            // setPower() clips the wheels at 1 instead.
            double max = Math.max(1, Math.max(Math.abs(pFwd), Math.max(Math.abs(pRight), Math.abs(pTurn))));
            if (max > 1){
                pBL /= max;
                pFL /= max;
                pFR /= max;
                pBR /= max;
            }
            bl.setPower(pBL);
            fl.setPower(pFL);
            fr.setPower(pFR);
            br.setPower(pBR);

        }

    }

}
