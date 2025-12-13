package org.firstinspires.ftc.teamcode.driver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.Core.FancyFormatting.*;

import org.firstinspires.ftc.teamcode.Core.RobotHardware;

/**
 *  Written by Written by <a href="https://github.com/AbsolutePug">Robert Maddox (AbsolutePug)</a> 2025
 *  <p>
 *  ROBOT Controller
 */
@TeleOp(name="TeleOp Controller", group="TeleOp")
public class RobotController extends LinearOpMode {
    RobotHardware robot = new RobotHardware();

    // Declare OpMode members for each of the motors
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime loop_time = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize the robot
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Init, Ready for Start!");
        telemetry.update();

        waitForStart();
        robot.setBrakes(RobotHardware.Brake.ENGAGED); // set the desired default state for brakes
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // Controller Inputs
            double axial   = gamepad1.left_stick_y;     // Forward
            double lateral = gamepad1.left_stick_x;     // Strafe
            double yaw     = gamepad1.right_stick_x;    // Turn
            double trigger = gamepad1.right_trigger;    // Right Trigger (Slow-mode)
            boolean flywheel_on = gamepad2.right_bumper;
            boolean flywheel_off = gamepad2.left_bumper;
            boolean feeder = gamepad2.x;
            boolean intake = gamepad2.a;
            boolean blocker_left = gamepad2.dpad_left;
            boolean blocker_right = gamepad2.dpad_right;
            boolean brakes_on = gamepad1.a;
            boolean brakes_off = gamepad1.b;

            // Flywheel Control
            if (flywheel_on) {
                robot.setFlywheel(true);
            }
            else if (flywheel_off) {
                robot.setFlywheel(false);
            }

            // Feeder
            robot.setFeeder(feeder);

            // Intake
            robot.setIntake(intake);


            // Blocker
            if (blocker_left) {
                robot.setBlocker(0.4);
            } else if (blocker_right) {
                robot.setBlocker(0.6);
            }

            // Speed multiplier. If right trigger is more than half way pressed half the motor power output
            double numerator = 1;
            if (trigger > 0.1) {
                numerator = 1 - (trigger / 2);
            }

            // Brake Control
            // If engaged the wheels will be locked in place, if not the wheels can be moved freely.
            if (brakes_on) {
                robot.setBrakes(RobotHardware.Brake.ENGAGED);
            }
            else if (brakes_off) {
                robot.setBrakes(RobotHardware.Brake.DISENGAGED);
            }

            double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            double leftFrontPower  = (axial - lateral - yaw)*numerator/denominator;
            double rightFrontPower = (axial + lateral + yaw)*numerator/denominator;
            double leftBackPower   = (axial + lateral - yaw)*numerator/denominator;
            double rightBackPower  = (axial - lateral + yaw)*numerator/denominator;

            // Normalize the values so no wheel power exceeds 100%. This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels
            robot.setPowers(
                    leftFrontPower,
                    rightFrontPower,
                    leftBackPower,
                    rightBackPower
            );

            // Basic Telemetry
            telemetry.addData           ("Status", runtime.toString());
            telemetry.addData           ("Loop time (ms)", loop_time.milliseconds());

            telemetry.addData           (toBoldString("CHASSIS"), "");
            telemetry.addData("Brakes", toStringBoolColored(robot.getBrakes()));
            telemetry.addData("Speed Multiplier", toStringPercent(numerator));

            telemetry.addData           (toBoldString("SCORING DEVICE"), "");
            telemetry.addData           ("Flywheel Velocity Raw", robot.getFlywheelVelocity());
            telemetry.addData("Flywheel Ready", toStringPercent(robot.getFlywheelVelocity()/robot.getFlywheelMaxVelocity()));

            telemetry.addData           (toBoldString("EXTRA"), "");
            telemetry.addData           ("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData           ("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
            loop_time.reset();
        }
    }
}