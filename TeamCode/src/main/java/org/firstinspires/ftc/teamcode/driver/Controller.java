package org.firstinspires.ftc.teamcode.driver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.driver.util.driveFunction;
import org.firstinspires.ftc.teamcode.driver.util.inputManager;

/*
 *  Written by Robert Maddox (github.com/AbsolutePug) 2025
 */

@TeleOp(name="TeleOp", group="!Custom")
public class Controller extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime loop_time = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    @Override
    public void runOpMode() {

        // Initialize the robot drive function manager with the correct motors
        driveFunction chassis = new driveFunction(
                hardwareMap.get(DcMotor.class, "leftFront"),
                hardwareMap.get(DcMotor.class, "leftBack"),
                hardwareMap.get(DcMotor.class, "rightFront"),
                hardwareMap.get(DcMotor.class, "rightBack"),
                hardwareMap.get(DcMotor.class, "launcher"),
                hardwareMap.get(CRServo.class, "mover1"),
                hardwareMap.get(CRServo.class, "mover2")
        );

        // Initialize inputs
        boolean last_gamepad_a = false;
        boolean last_gamepad_1_right_bumper = false;
        boolean last_gamepad_2_right_bumper = false;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Init, Ready for Start!");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // Set zero power behaviors
        chassis.setBrakes(true);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // Controller Inputs
            double axial   = gamepad1.left_stick_y;  // Forward
            double lateral = gamepad1.left_stick_x;  // Strafe
            double yaw     = -gamepad1.right_stick_x; // Turn
            double trigger = gamepad1.right_trigger;  // Right Trigger (Slow-mode)

            // Launcher Control
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                chassis.setLauncher(true);
            } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
                chassis.setLauncher(false);
            }

            chassis.setMover(gamepad1.x || gamepad2.x);

            // Speed multiplier. If right trigger is more than half way pressed half the speed output
            double speed_coefficient = 1;
            if (trigger > 0.1) {
                speed_coefficient = 1 - (trigger / 2);
            }

            // Brake Control
            // If engaged the wheels will be locked in place, if not the wheels can be moved freely.
            if (gamepad1.a  && !last_gamepad_a) {
                chassis.setBrakes();
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
            chassis.setPower(
                    leftFrontPower,
                    rightFrontPower,
                    leftBackPower,
                    rightBackPower
            );

            // Update controller inputs
            last_gamepad_a = gamepad1.a;

            // Basic Telemetry
            telemetry.addData("Status", runtime.toString());
            telemetry.addData("Loop time (ms)", loop_time.milliseconds());
            // Brake Telemetry
            if (chassis.getBrakes())  {telemetry.addData("Brake", "Engaged");}
            else        {telemetry.addData("Brake", "Disengaged");}
            telemetry.addData("Speed Multiplier", speed_coefficient);
            //
            telemetry.addData("-- Scoring Device --", "");
            telemetry.addData("Launcher Active",chassis.getLauncher());

            // Misc
            telemetry.addData("-- Misc --", ""); //Divider
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();

            loop_time.reset();
        }
    }
}