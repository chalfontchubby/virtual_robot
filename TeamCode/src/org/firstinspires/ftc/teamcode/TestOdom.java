package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorExImpl;

/**
 * TeleOp op mode to test odometry with three "dead-wheel" encoders. This op mode will work with
 * either the Mecanum Bot or the XDrive Bot robot configuration.
 *
 * Dead wheels are small unpowered wheels that roll along the floor, each with an encoder.
 * Unlike the drive wheels they don't slip under power, so they measure how far the robot has
 * really moved. All the hardware setup and odometry maths are in the EncBot class.
 *
 * Controls (gamepad1):
 *   left stick      drive forward/back and strafe left/right
 *   left trigger    turn left
 *   right trigger   turn right
 */
@TeleOp(name = "TestOdom", group = "OdomBot")
public class TestOdom extends LinearOpMode {

    // Keeping the hardware in a separate class lets several OpModes share the same setup code.
    EncBot bot = new EncBot();
    double[] pose;

    public void runOpMode(){
        bot.init(hardwareMap);
        // Start at x = 0, y = 0, facing along the +Y axis (90 degrees).
        bot.resetOdometry(0, 0, Math.PI/2.0);

        // Does the same as waitForStart(), but keeps updating telemetry while it waits.
        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("time", this.time);
            telemetry.update();
        }

        while (opModeIsActive()){
            // Odometry works in small steps: each call adds the movement since the previous
            // call, so it must run every loop. pose is {x, y, heading in radians}.
            pose = bot.updateOdometry();
            telemetry.addData("time", this.time);
            telemetry.addData("POSE", "x = %.1f  y = %.1f  h = %.1f", pose[0], pose[1],
                    Math.toDegrees(pose[2]));
            // T = encoder ticks, V = velocity in ticks per second.
            telemetry.addData("Back Left", "T = %d  V = %.0f", bot.motors[0].getCurrentPosition(), bot.motors[0].getVelocity());
            telemetry.addData("Front Left", "T = %d  V = %.0f", bot.motors[1].getCurrentPosition(), bot.motors[1].getVelocity());
            telemetry.addData("Front Right", "T = %d  V = %.0f", bot.motors[2].getCurrentPosition(), bot.motors[2].getVelocity());
            telemetry.addData("Back Right", "T = %d  V = %.0f", bot.motors[3].getCurrentPosition(), bot.motors[3].getVelocity());
            telemetry.addData("Right Enc", "T = %d  V = %.0f", bot.encoders[0].getCurrentPosition(), bot.encoders[0].getVelocity());
            telemetry.addData("Left Enc", "T = %d  V = %.0f", bot.encoders[1].getCurrentPosition(), bot.encoders[1].getVelocity());
            telemetry.addData("X Enc", "T = %d  V = %.0f", bot.encoders[2].getCurrentPosition(), bot.encoders[2].getVelocity());
            telemetry.update();

            // px = strafe right, py = forward, pa = turn left. Stick y is negative when pushed
            // forward, hence the minus sign.
            double px = gamepad1.left_stick_x;
            double py = -gamepad1.left_stick_y;
            double pa = gamepad1.left_trigger - gamepad1.right_trigger;
            bot.setDrivePower(px, py, pa);
        }
    }
}
