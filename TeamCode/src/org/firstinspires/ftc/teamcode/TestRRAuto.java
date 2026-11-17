package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.lang.Math;

/**
 * Example autonomous using RoadRunner 1.0: drive a spline path across the field, raise an arm and
 * move a claw, then drive back and put the arm down.
 *
 * Robot configuration: MecDynamic Bot (it has the drive, Pinpoint, arm motor and hand servo this
 * needs). Press INIT, then START; the robot drives itself.
 *
 * RoadRunner works in field coordinates, in inches, with (0, 0) at the centre of the field.
 * A Pose2d is (x, y, heading in radians); heading 0 faces along +X.
 *
 * The key idea is the Action. An Action is one step of the autonomous (follow a path, move the
 * arm...). RoadRunner calls its run() method over and over; run() does a little work and returns
 * true to say "call me again" or false to say "I'm finished". Actions can be combined, e.g. a
 * SequentialAction runs a list of them one after another.
 */
@Autonomous(name = "Test RR Auto", group = "RR")
public class TestRRAuto extends LinearOpMode {

    /**
     * The arm and claw, plus the Actions that move them. Keeping the hardware and its Actions
     * together in one class is the pattern the RoadRunner docs recommend.
     */
    public class Arm{

        private DcMotorEx armMotor;
        private Servo clawServo;

        public Arm(HardwareMap hardwareMap){
            armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
            // RUN_TO_POSITION: the motor controller drives the motor to a target encoder count
            // and holds it there. The target must be set before switching to this mode.
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setTargetPosition(0);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            clawServo = hardwareMap.get(Servo.class, "hand_servo");
        }

        /**
         * Action that moves the arm to a target encoder position, and finishes once it gets there.
         */
        public class SetArmPosition implements Action{
            // run() is called many times; only set the target on the first call.
            private boolean initialized = false;
            private int targetPosition;

            public SetArmPosition(int targetPosition){
                this.targetPosition = targetPosition;
            }

            public boolean run(TelemetryPacket packet){
                if (!initialized){
                    initialized = true;
                    armMotor.setTargetPosition(targetPosition);
                    armMotor.setPower(1);
                }
                // isBusy() is true until the motor reaches its target, so keep running till then.
                return armMotor.isBusy();
            }
        }
        // Helper so the sequence below can say arm.setPosition(2000) instead of
        // arm.new SetArmPosition(2000).
        public Action setPosition(int targetPosition){
            return new SetArmPosition(targetPosition);
        }

        /**
         * Action that sets the claw servo's position. It finishes straight away, without
         * waiting for the servo to physically get there.
         */
        public class SetClawPosition implements Action{
            double position;

            public SetClawPosition(double position){
                this.position = position;
            }

            public boolean run(TelemetryPacket packet){
                clawServo.setPosition(position);
                // false = finished after one call.
                return false;
            }
        }
        public Action setClawPosition(double position){
            return new SetClawPosition(position);
        }
    }

    public void runOpMode(){

        // MecanumDrive is RoadRunner's drive class (in the roadrunner package). Give it the
        // robot's starting pose on the field.
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-24, 24, 0));
        Arm arm = new Arm(hardwareMap);

        // Build the paths during init so START isn't delayed. Each actionBuilder() starts at
        // the pose where the robot will be when that path begins.
        // splineTo(point, tangent) drives a smooth curve to the point, arriving travelling in
        // the direction of the tangent (0 = along +X).
        Action driveForward = drive.actionBuilder(new Pose2d(-24, 24, 0))
                        .setReversed(false)
                        .splineTo(new Vector2d(24, -24), 0)
                        .build();

        // setReversed(true) drives the path backwards, so the robot reverses along it rather
        // than turning around. The tangent is the direction of travel, which while reversing is
        // opposite to the way the robot faces, so Math.PI here means "moving towards -X".
        Action driveReverse = drive.actionBuilder(new Pose2d(24, -24,0))
                        .setReversed(true)
                        .splineTo(new Vector2d(-24, 24), Math.PI)
                        .build();


        waitForStart();

        // Run the steps one after another. runBlocking() keeps calling run() on the current
        // Action until the whole sequence is finished.
        Actions.runBlocking(
                new SequentialAction(
                        driveForward,
                        arm.setPosition(2000),
                        arm.setClawPosition(1),
                        driveReverse,
                        arm.setClawPosition(0),
                        arm.setPosition(0)
                )
        );

    }

}
