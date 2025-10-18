package org.firstinspires.ftc.teamcode.driver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.RobotHardware;
import org.firstinspires.ftc.teamcode.Core.telemetrySophisticated;

/**
 *  Written by Written by <a href="https://github.com/AbsolutePug">Robert Maddox (AbsolutePug)</a> 2025
 *  <p>
 *  ROBOT Controller
 */
@TeleOp(name="TeleOp Controller", group="TeleOp")
public class RobotController extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    telemetrySophisticated telemetryAdvanced = new telemetrySophisticated(telemetry); // Pass telemetry to super awesome version of telemetry. I know how to program

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
        runtime.reset();

        // Set initial zero power behaviors
        robot.setBrakes(true);

        // Set directions using

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // Controller Inputs
            double axial   = gamepad1.left_stick_y;     // Forward
            double lateral = gamepad1.left_stick_x;     // Strafe
            double yaw     = -gamepad1.right_stick_x;   // Turn
            double trigger = gamepad1.right_trigger;    // Right Trigger (Slow-mode)

            // Flywheel Control
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                robot.setFlywheel(true);
            }
            else if (gamepad1.left_bumper || gamepad2.left_bumper) {
                robot.setFlywheel(false);
            }

            if (robot.getFlywheelReady()) {
                if (gamepad1.x || gamepad2.x) {
                    robot.setGuider(true);
                } // The guiders should not respond to inputs if the flywheel is inactive
                else {
                    robot.setGuider(-0.1);
                }
            }
            else {
                robot.setGuider(false);
            }

            // Speed multiplier. If right trigger is more than half way pressed half the speed output
            double speed_coefficient = 1;
            if (trigger > 0.1) {
                speed_coefficient = 1 - (trigger / 2);
            }

            // Brake Control
            // If engaged the wheels will be locked in place, if not the wheels can be moved freely.
            if (gamepad1.a) {
                robot.setBrakes(true);
            }
            else if (gamepad1.b) {
                robot.setBrakes(false);
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            double leftFrontPower  = (axial - lateral - yaw)*speed_coefficient;
            double rightFrontPower = (axial + lateral + yaw)*speed_coefficient;
            double leftBackPower   = (axial + lateral - yaw)*speed_coefficient;
            double rightBackPower  = (axial - lateral + yaw)*speed_coefficient;

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
            robot.setPower(
                    leftFrontPower,
                    rightFrontPower,
                    leftBackPower,
                    rightBackPower
            );

            // Basic Telemetry
            telemetry.addData           ("Status", runtime.toString());
            telemetry.addData           ("Loop time (ms)", loop_time.milliseconds());

            telemetry.addData           ("\uD83C\uDDE8\u200C\uD83C\uDDED\u200C\uD83C\uDDE6\u200C\uD83C\uDDF8\u200C\uD83C\uDDF8\u200C\uD83C\uDDEE\u200C\uD83C\uDDF8\u200C", ""); // "Chassis"
            telemetryAdvanced.addBoolean("Brakes", robot.getBrakes());
            telemetryAdvanced.addPercent("Speed Multiplier", speed_coefficient);

            telemetry.addData           ("\uD83C\uDDF8\u200C\uD83C\uDDE8\u200C\uD83C\uDDF4\u200C\uD83C\uDDF7\u200C\uD83C\uDDEE\u200C\uD83C\uDDF3\u200C\uD83C\uDDEC\u200C \uD83C\uDDE9\u200C\uD83C\uDDEA\u200C\uD83C\uDDFB\u200C\uD83C\uDDEE\u200C\uD83C\uDDE8\u200C\uD83C\uDDEA\u200C", ""); // "Scoring device"
            telemetryAdvanced.addPercent("Flywheel Ready", robot.getFlywheelTime());

            telemetry.addData("\uD83C\uDDEA\u200C\uD83C\uDDFD\u200C\uD83C\uDDF9\u200C\uD83C\uDDF7\u200C\uD83C\uDDE6\u200C", ""); //Divider
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
            loop_time.reset();
        }
    }
}