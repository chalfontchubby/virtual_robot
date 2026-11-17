package org.firstinspires.ftc.teamcode;

// "import" lines tell Java where to find the SDK classes this file uses. Blocks did this for
// you. In IntelliJ or Android Studio, typing a class name and pressing Alt+Enter adds the import.
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * A first Java OpMode for teams coming from Blocks.
 *
 * It does what a typical Blocks TeleOp does: drive a mecanum robot with the gamepad, move a
 * servo with buttons, and show the IMU heading in telemetry. Each section is labelled with the
 * blocks it replaces. The structure is exactly the Blocks LinearOpMode template:
 * set up hardware, waitForStart, then "repeat while opModeIsActive".
 *
 * Robot configuration: Mecanum Bot.
 *
 * Controls (gamepad1):
 *   left stick      drive forward/back and strafe left/right
 *   right stick X   turn
 *   Y / A           back servo up / down
 *
 * Things Java needs that Blocks hid from you:
 *   - Every statement ends with a semicolon ;
 *   - Curly braces { } mark where a block of code starts and ends, like the "mouth" of a
 *     Blocks "if" or "repeat" block.
 *   - Capitals matter: setPower is not the same as SetPower.
 *   - Every variable has a type, declared when you create it (see below).
 *
 * Try it: change something small (a servo position, the speed limit), run it, and see what happens.
 */

// @TeleOp puts this OpMode in the TeleOp list on the Driver Station (or the simulator's OpModes
// dropdown) under this name. In Blocks you chose TeleOp or Autonomous when creating the OpMode;
// for an autonomous, use @Autonomous instead.
@TeleOp(name = "Blocks to Java", group = "Learning")

// "extends LinearOpMode" means this class is a LinearOpMode: it gets waitForStart(),
// opModeIsActive(), gamepad1, telemetry, hardwareMap and the rest from the SDK for free.
// The class name must match the file name: BlocksToJava.java.
public class BlocksToJava extends LinearOpMode {

    // ---- Variables ----
    // In Blocks, the Variables drawer held any kind of value. In Java, each variable has a
    // type, which says what it can hold:
    //   DcMotor, Servo, IMU   a piece of hardware
    //   double                a decimal number, such as 0.5 or -1.0
    //   int                   a whole number, such as 3 or -120
    //   boolean               true or false
    // Declaring them here, outside any method, lets every method in this class use them.
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Servo backServo;
    private IMU imu;

    // A constant: "final" means it can't be changed while the program runs. Capitals are the
    // Java convention for constants.
    private final double SPEED_LIMIT = 0.8;

    // runOpMode() is the code that runs when you press INIT. It's everything that was inside
    // the "to runOpMode" block in Blocks. "@Override" says we're filling in a method that
    // LinearOpMode expects us to provide.
    @Override
    public void runOpMode() {

        // ---- Hardware setup ----
        // Blocks: the motor and servo names in the drop-downs come from the robot configuration.
        // Java: hardwareMap.get(type, "name"). The name must match the configuration exactly,
        // or the OpMode stops with an error when you press INIT.
        frontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight = hardwareMap.get(DcMotor.class, "back_right_motor");
        backServo = hardwareMap.get(Servo.class, "back_servo");
        imu = hardwareMap.get(IMU.class, "imu");

        // Blocks: set frontLeft.Direction to REVERSE
        // The left motors face the opposite way to the right ones, so reverse them to make
        // positive power drive every wheel forward.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Blocks: set frontLeft.ZeroPowerBehavior to BRAKE
        // The motors stop sharply when the sticks are released, rather than coasting.
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Blocks: call imu.initialize, with the "new IMU.Parameters" and
        // "RevHubOrientationOnRobot" blocks plugged in.
        // "new" creates an object; here, the description of how the hub is mounted.
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        // Blocks: call telemetry.addData, then call telemetry.update
        telemetry.addData("Status", "Ready. Press START.");
        telemetry.update();

        // Blocks: call waitForStart
        waitForStart();

        // Blocks: repeat while call opModeIsActive
        // "while" repeats everything between its { and } for as long as the condition is true.
        while (opModeIsActive()) {

            // ---- Read the gamepad ----
            // Blocks: gamepad1.LeftStickY
            // The stick's Y is -1 when pushed forward, so put a minus in front to make
            // forward positive.
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // ---- Drive ----
            // Instead of a long run of setPower blocks here, call our own method, defined below
            // (like a Blocks "function"), and pass it the three values.
            driveMecanum(forward, strafe, turn);

            // ---- Servo ----
            // Blocks: if gamepad1.Y do ... else if gamepad1.A do ...
            // A gamepad button is a boolean: true while held down.
            if (gamepad1.y) {
                backServo.setPosition(1.0);
            } else if (gamepad1.a) {
                backServo.setPosition(0.0);
            }

            // ---- Telemetry ----
            // Blocks: call imu.getRobotYawPitchRollAngles, then YawPitchRollAngles.getYaw
            // Java lets you join the two calls with dots.
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // "%.1f" means "show this number with 1 decimal place".
            telemetry.addData("Heading", "%.1f degrees", heading);
            telemetry.addData("Sticks", "fwd %.2f  strafe %.2f  turn %.2f", forward, strafe, turn);
            telemetry.addData("Servo", backServo.getPosition());
            telemetry.update();
        }
    }

    /**
     * Drive a mecanum robot. This is our own method, the Java version of a Blocks function.
     * "private" means only this class can use it. "void" means it doesn't hand back a value.
     * The three values in brackets are its parameters: what it needs to be told each time.
     *
     * @param forward  positive = forward, -1 to 1
     * @param strafe   positive = right, -1 to 1
     * @param turn     positive = clockwise (turn right), -1 to 1
     */
    private void driveMecanum(double forward, double strafe, double turn) {
        // Each wheel's power is a sum of the three movements. To strafe right, the front-left
        // and back-right wheels go forward while the other two go backward. To turn right,
        // the left side goes forward and the right side backward.
        double frontLeftPower = forward + strafe + turn;
        double frontRightPower = forward - strafe - turn;
        double backLeftPower = forward - strafe + turn;
        double backRightPower = forward + strafe - turn;

        // Those sums can be bigger than 1, which a motor can't do. Find the biggest of the
        // four (Blocks: the Math drawer's "max" and "absolute" blocks). If it's over 1, divide
        // all four by it so they keep the same proportions and the robot still goes the way
        // you asked.
        double biggest = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        biggest = Math.max(biggest, Math.abs(backLeftPower));
        biggest = Math.max(biggest, Math.abs(backRightPower));
        if (biggest > 1.0) {
            frontLeftPower = frontLeftPower / biggest;
            frontRightPower = frontRightPower / biggest;
            backLeftPower = backLeftPower / biggest;
            backRightPower = backRightPower / biggest;
        }

        // Blocks: call frontLeft.setPower
        frontLeft.setPower(frontLeftPower * SPEED_LIMIT);
        frontRight.setPower(frontRightPower * SPEED_LIMIT);
        backLeft.setPower(backLeftPower * SPEED_LIMIT);
        backRight.setPower(backRightPower * SPEED_LIMIT);
    }
}
