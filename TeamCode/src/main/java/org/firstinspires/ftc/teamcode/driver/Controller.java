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
    private DcMotor arm = null;
    private Servo wrist = null;
    private CRServo claw = null;
    driveFunction chassis = new driveFunction(
            hardwareMap.get(DcMotor.class, "leftFront"),
            hardwareMap.get(DcMotor.class, "leftBack"),
            hardwareMap.get(DcMotor.class, "rightFront"),
            hardwareMap.get(DcMotor.class, "rightBack"),
            hardwareMap.get(DcMotor.class, "arm"),
            hardwareMap.get(Servo.class, "wrist"),
            hardwareMap.get(CRServo.class, "claw")
    ); // horray
    inputManager inputs = new inputManager(); // Initialize input manager

    @Override
    public void runOpMode() {


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Init, Ready for Start!");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // Set zero power behaviors
        chassis.setBrakes(true);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // DEPLOY ARM
        chassis.armDeploy();
        chassis.setClaw(-1);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // Controller Inputs
            double axial   = gamepad1.left_stick_y;  // Forward
            double lateral = gamepad1.left_stick_x;  // Strafe
            double yaw     = gamepad1.right_stick_x; // Turn
            double trigger = gamepad1.right_trigger;  // Right Trigger (Slow-mode)
            // Scoring device
            double arm_y   = -gamepad2.right_stick_y; // Up/Down

            // Scoring device
            double arm_power = arm_y*arm_y; // Exponential arm power
            if (-gamepad2.right_stick_y < 0) {
                arm_power = -arm_power;
            }
            chassis.setArmPower(arm_power);

            // Wrist Control
            if (gamepad2.dpad_left)                             {chassis.setWrist(chassis.arm_x_left);  }
            else if (gamepad2.dpad_right)                       {chassis.setWrist(chassis.arm_x_right); }
            else if (gamepad2.dpad_up || gamepad2.dpad_down)    {chassis.setWrist(chassis.arm_x_center);}

            // Claw control
            if (inputs.leftBumperPressed()) {
                chassis.setClaw(1);
            }
            if (inputs.rightBumperPressed()) {
                chassis.setClaw(-1);
            }

            // Hang
            if (gamepad2.y) {
                chassis.autoHang();
            }

            // Speed multiplier. If right trigger is more than half way pressed half the speed output
            double speed_coefficient;
            if (trigger > 0.5) {
                speed_coefficient = 0.5;
            }
            else {
                speed_coefficient = 1;
            }

            // Brake Control
            // If engaged the wheels will be locked in place, if not the wheels can be moved freely.
            if (inputs.aPressed()) {
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

            // Basic Telemetry
            telemetry.addData("Status", runtime.toString());
            telemetry.addData("Loop time (ms)", loop_time.milliseconds());
            // Brake Telemetry
            if (chassis.getBrakes())  {telemetry.addData("Brake", "Engaged");}
            else        {telemetry.addData("Brake", "Disengaged");}
            telemetry.addData("Speed Multiplier", speed_coefficient);
            //
            telemetry.addData("-- Scoring Device --", "");
            if (chassis.getArm()) {
                telemetry.addData("Arm Power", arm_y);
                //telemetry.addData("Wrist Position", wrist_x); not storing wrist position anymore
            } else {
                telemetry.addData("Arm", "LOCKED");
            }

            // Misc
            telemetry.addData("-- Misc --", ""); //Divider
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();

            loop_time.reset();

            inputs.updateInputs();
        }
    }
}