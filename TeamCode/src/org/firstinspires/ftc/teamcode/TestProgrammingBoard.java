package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Exercises every device on the Programming Board, the stationary test board from the book
 * "Learn Java For FTC" by Alan Smith. The board doesn't drive, but you can rotate it by dragging
 * it with the mouse to see the IMU heading change.
 *
 * This is an iterative OpMode: the SDK calls init() once when INIT is pressed, then loop()
 * repeatedly from START until STOP. Telemetry is sent automatically after each call.
 *
 * Robot configuration: Programming Board.
 *
 * Controls (gamepad1):
 *   left stick Y    motor power
 *   right stick Y   servo position
 */
@TeleOp(name = "TestProgrammingBoard", group = "Test")
public class TestProgrammingBoard extends OpMode {

    DcMotor motor = null;
    Servo servo = null;
    BNO055IMU imu = null;
    ColorSensor colorSensor = null;
    AnalogInput analogInput = null;
    DigitalChannel digitalChannel = null;
    DistanceSensor distanceSensor = null;

    public void init(){
        motor = hardwareMap.get(DcMotor.class, "motor");

        servo = hardwareMap.get(Servo.class, "servo");
        // Servo positions run from 0 to 1; start in the middle.
        servo.setPosition(0.5);

        // One physical sensor, the REV Color/Distance sensor, is read through two interfaces,
        // so both lookups use the same name.
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        // The potentiometer is read as a voltage on an analog port.
        analogInput = hardwareMap.get(AnalogInput.class, "pot");
        // The touch sensor is a digital (on/off) input.
        digitalChannel = hardwareMap.get(DigitalChannel.class, "touch_sensor");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

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

        // Show the motor's specifications (gear ratio, encoder ticks per revolution, top speed).
        MotorConfigurationType motorConfigType = motor.getMotorType();
        telemetry.addData("Motor", " MaxRPM = %.2f  MaxRPMFraction = %.2f", motorConfigType.getMaxRPM(),
                motorConfigType.getAchieveableMaxRPMFraction());
        telemetry.addData("Motor", " AchieveableMaxTicksPerSec = %.2f", motorConfigType.getAchieveableMaxTicksPerSecond());
        telemetry.addData("Motor ", " Ticks/Rot = %.2f  gearing = %.2f", motorConfigType.getTicksPerRev(),
                motorConfigType.getGearing());
        telemetry.addData("Motor Orientation", motorConfigType.getOrientation());

    }


    public void loop(){
        // Stick y is negative when pushed forward, so negate it: forward = positive power.
        motor.setPower(-gamepad1.left_stick_y);
        // Map the stick's -1..+1 onto the servo's 0..1: forward = 0, centred = 0.5, back = 1.
        servo.setPosition(0.5*(gamepad1.right_stick_y + 1));
        // With the default axis order, firstAngle is the heading (rotation about Z).
        Orientation orientation = imu.getAngularOrientation();
        telemetry.addData("Heading", "%.1f deg", orientation.firstAngle);
        telemetry.addData("Ticks", motor.getCurrentPosition());
        telemetry.addData("Volts", analogInput.getVoltage());
        // A touch sensor reads true when NOT pressed, so invert it to get "is pressed".
        telemetry.addData("Touch", !digitalChannel.getState());
        telemetry.addData("Color", "R: %d  G: %d  B: %d", colorSensor.red(),
                colorSensor.green(), colorSensor.blue());
        telemetry.addData("Distance (CM)", distanceSensor.getDistance(DistanceUnit.CM));

    }

}
