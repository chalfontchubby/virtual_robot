package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;

/**
 * Drive the robot around and watch what the SparkFun OTOS (Optical Tracking Odometry Sensor)
 * reports: position, velocity and acceleration. The OTOS tracks the robot with a downward-facing
 * optical sensor (like a computer mouse) plus its own built-in IMU.
 *
 * Robot configuration: Mecanum Bot, MecDynamic Bot or XDrive Bot (the robots with a "sensor_otos").
 *
 * Controls (gamepad1):
 *   left stick      drive forward/back and strafe left/right
 *   right stick X   turn
 *   A               set the OTOS position to (0, 0), heading 0
 *   B               resetTracking(), which in the simulator does the same as A
 */
@TeleOp
public class TestOTOS extends LinearOpMode {

    SparkFunOTOS otos;
    DcMotorEx bl, fl, fr, br;

    public void runOpMode(){

        // ---- Init: everything up to waitForStart() runs when INIT is pressed ----

        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

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

        // Units for every value the OTOS reports from here on.
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        // On a real robot, the scalars correct for distance and rotation errors found during
        // tuning, and the offset says where the sensor is mounted relative to the robot's centre.
        // The simulator stores these values but doesn't apply them.
        otos.setAngularScalar(1.0);
        otos.setLinearScalar(1.0);
        otos.setOffset(new Pose2D(0, 0, 0));
        // Calibrate the built-in IMU. The robot must be stationary while this runs.
        otos.calibrateImu();

        // Show the OTOS's starting readings while waiting for START.
        Pose2D otosPos = otos.getPosition();
        Pose2D otosVel = otos.getVelocity();
        Pose2D otosAccel = otos.getAcceleration();

        telemetry.addData("Otos Pose", "x: %.1f  y: %.1f  h: %.1f", otosPos.x, otosPos.y, otosPos.h);
        telemetry.addData("Otos Vel", "vx: %.1f  vy: %.1f  vh: %.1f", otosVel.x, otosVel.y, otosVel.h);
        telemetry.addData("Otos Accel", "ax: %.1f  ay: %.1f  ah: %.1f", otosAccel.x, otosAccel.y, otosAccel.h);

        telemetry.update();


        // Wait here until START is pressed.
        waitForStart();

        // ---- Run: loop until STOP is pressed ----
        while (opModeIsActive()){

            // Make wherever the robot is now the new origin.
            if (gamepad1.a){
                otos.setPosition(new Pose2D(0, 0, 0));
            } else if (gamepad1.b){
                otos.resetTracking();
            }

            // Unlike the Pinpoint, the OTOS has no update() call: each get...() reads the latest
            // values. Velocity and acceleration are in the same coordinate frame as the position,
            // not relative to the robot.
            otosPos = otos.getPosition();
            otosVel = otos.getVelocity();
            otosAccel = otos.getAcceleration();

            telemetry.addData("Otos Pose", "x: %.1f  y: %.1f  h: %.1f", otosPos.x, otosPos.y, otosPos.h);
            telemetry.addData("Otos Vel", "vx: %.1f  vy: %.1f  vh: %.1f", otosVel.x, otosVel.y, otosVel.h);
            telemetry.addData("Otos Accel", "ax: %.1f  ay: %.1f  ah: %.1f", otosAccel.x, otosAccel.y, otosAccel.h);

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
