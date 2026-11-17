package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Example TeleOp for a two-wheel (differential drive) robot, written as an iterative OpMode.
 * Demonstrates driving with buttons, positioning a servo, and reading the IMU, color sensor,
 * encoders and distance sensors.
 *
 * An iterative OpMode doesn't have a runOpMode() method with its own loop. Instead the SDK calls:
 *   init()       once, when INIT is pressed
 *   init_loop()  repeatedly, between INIT and START
 *   loop()       repeatedly, from START until STOP
 * Each method must return quickly (no long loops or sleeps), and telemetry is sent
 * automatically after each call, so there is no telemetry.update() here.
 *
 * Robot configuration: Two Wheel Bot.
 *
 * Controls (gamepad1):
 *   Y / A           drive forward / backward
 *   B / X           spin right / left on the spot
 *   left stick Y    move the back servo
 */
@TeleOp(name = "two wheel demo opmode", group = "TwoWheel")
public class TwoWheelDemoOpMode extends OpMode {

    private DcMotor left = null;
    private DcMotor right = null;
    private BNO055IMU imu = null;
    private Servo backServo = null;
    private ColorSensor colorSensor = null;
    private DistanceSensor frontDistance = null;
    private DistanceSensor leftDistance = null;
    private DistanceSensor backDistance = null;
    private DistanceSensor rightDistance = null;

    // Counts the seconds spent waiting for START, shown during init_loop().
    private ElapsedTime et = null;
    private int waitForStartTime = 0;

    public void init(){
        // Device names must match those in the robot configuration.
        left = hardwareMap.dcMotor.get("left_motor");
        right = hardwareMap.dcMotor.get("right_motor");
        // The left motor faces the opposite way to the right, so reverse it to make positive
        // power drive both wheels forward.
        left.setDirection(DcMotor.Direction.REVERSE);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        backServo = hardwareMap.servo.get("back_servo");
        colorSensor = hardwareMap.colorSensor.get("color_sensor");
        frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");

        // The older BNO055IMU interface must be initialized with parameters before use.
        // Only the units matter here; the rest switch off features that aren't needed.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";
        imu.initialize(parameters);

        et = new ElapsedTime();
    }

    public void init_loop(){
        // Add one to the counter every second.
        if (et.milliseconds() >= 1000) {
            waitForStartTime++;
            et.reset();
        }
        telemetry.addData("Press Start to Continue"," %d", waitForStartTime);
    }

    public void loop(){
        // Equal power on both wheels drives straight. Opposite powers spin the robot on the
        // spot. If no button is held, stop.
        if (gamepad1.a){
            telemetry.addData("a pressed","");
            left.setPower(-.5);
            right.setPower(-.5);
        } else if (gamepad1.y) {
            telemetry.addData("y pressed", "");
            left.setPower(0.5);
            right.setPower(0.5);
        } else if (gamepad1.b){
            telemetry.addData("b pressed", "");
            left.setPower(0.5);
            right.setPower(-0.5);
        } else if (gamepad1.x){
            telemetry.addData("x pressed", "");
            left.setPower(-0.5);
            right.setPower(0.5);
        } else {
            left.setPower(0);
            right.setPower(0);
        }
        // Servo positions run from 0 to 1. This maps the stick's -1..+1 onto that range:
        // stick centred = 0.5, pushed forward (y = -1) = 1, pulled back (y = +1) = 0.
        backServo.setPosition(0.5 - 0.5* gamepad1.left_stick_y);
        telemetry.addData("Press", "Y-fwd, A-rev, B-Rt, X-Lt");
        telemetry.addData("Left Gamepad stick controls back servo","");
        telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
        // With ZYX axis order, firstAngle is rotation about Z, i.e. the robot's heading.
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading"," %.1f", orientation.firstAngle);
        telemetry.addData("Encoders","Left %d  Right %d", left.getCurrentPosition(), right.getCurrentPosition());
        telemetry.addData("Distance", " Fr %.1f  Lt %.1f  Rt %.1f  Bk %.1f  ",
                frontDistance.getDistance(DistanceUnit.CM), leftDistance.getDistance(DistanceUnit.CM),
                rightDistance.getDistance(DistanceUnit.CM), backDistance.getDistance(DistanceUnit.CM)
        );

    }

}
