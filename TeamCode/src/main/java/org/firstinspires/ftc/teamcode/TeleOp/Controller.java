package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
/*
    Written by Robert Maddox (2025) @AbsolutePug (github.com/AbsolutePug)
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

    // Constants
    final double arm_x_right = 0.18;
    final double arm_x_center = 0.5;
    final double arm_x_left = 0.82;

    // Variables
    boolean arm_active = false; // Controls whether arm can be controlled by user input
    boolean arm_busy = false; // Is the arm currently changing state?
    double wrist_x = arm_x_left;

    // Inputs
    public boolean gamepad_a = false;
    boolean gamepad_a_last = false;
    boolean gamepad_right_bumper_last = false;
    boolean gamepad_left_bumper_last = false;
    boolean Input_brake = false;
    int Input_claw = 0;

    void updateInputs() {
        gamepad_a = gamepad1.a;

        // Input
        Input_brake = gamepad_a && !gamepad_a_last;
        if (gamepad2.right_bumper && !gamepad_right_bumper_last) {
            if (Input_claw == -1) {Input_claw = 0;}
            else                 {Input_claw = -1;}
        }
        if (gamepad2.left_bumper && !gamepad_left_bumper_last) {
            if (Input_claw == 1) {Input_claw = 0;}
            else                  {Input_claw = 1;}
        }

        // Store the input recorded this update
        gamepad_a_last = gamepad_a;
        gamepad_left_bumper_last  = gamepad2.left_bumper;
        gamepad_right_bumper_last = gamepad2.right_bumper;
    } // Method to update inputs
    void armDeploy() {
        ElapsedTime deploy_timeout = new ElapsedTime();
        deploy_timeout.reset();
        arm_active = false;
        arm_busy = true;

        while (deploy_timeout.milliseconds() < 1000) {
            // Arm
            if (deploy_timeout.milliseconds() < 500) {
                Input_claw = -1;
                claw.setPower(Input_claw);
                arm.setPower(1);
            } // Move up for first 500ms
            else {
                arm.setPower(-1);
            } // Move down for 500ms after 500ms

            // Servo
            if (deploy_timeout.milliseconds() > 500)  {
                wrist_x = arm_x_center;
                wrist.setPosition(wrist_x);
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } // Move servo after moving arm up
        }
        // Arm has finished changing state
        arm_active = true;
        arm_busy = false;
    } // Deploy arm into ready position
    void dtSetPower(double FL, double FR, double BL, double BR) {
        leftFront.setPower(-FL);
        rightFront.setPower(-FR);
        leftBack.setPower(-BL);
        rightBack.setPower(-BR);
    } // Set the power of each motor
    void autoHang() {
        ElapsedTime hang_timeout = new ElapsedTime();
        hang_timeout.reset();
        arm_active = false;
        arm_busy = true;

        while (opModeIsActive()) {
            // Drive
            if (hang_timeout.milliseconds() > 1500 && hang_timeout.milliseconds() < 2500) {
                dtSetPower(0.5,0.5,0.5,0.5);
            }

            // Arm
            if (hang_timeout.milliseconds() < 1600) {
                arm.setPower(1);
            } else {
                arm.setPower(-1);
            }
        }
    } // Completes the ascent procedure

    @Override
    public void runOpMode() {
        // Initialize the hardware variables.
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        arm = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.scaleRange(0,1);
        claw = hardwareMap.get(CRServo.class, "claw");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Init, Ready for Start!");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // Set zero power behaviors
        boolean brake = true;
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wrist.setPosition(arm_x_left);

        armDeploy();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // Controller Inputs
            updateInputs();
            double axial   = gamepad1.left_stick_y;  // Forward
            double lateral = gamepad1.left_stick_x;  // Strafe
            double yaw     = gamepad1.right_stick_x; // Turn
            double trigger = gamepad1.right_trigger;  // Right Trigger (Slow-mode)
            // Scoring device
            double arm_y   = -gamepad2.right_stick_y; // Up/Down

            // Scoring device
            if (arm_active) {
                double arm_power = arm_y*arm_y;
                if (-gamepad2.right_stick_y < 0) {arm_power = -arm_power;}
                arm.setPower(arm_power);

                // Wrist Control
                if (gamepad2.dpad_left) {wrist_x = arm_x_left;}
                else if (gamepad2.dpad_right) {wrist_x = arm_x_right;}
                else if (gamepad2.dpad_up || gamepad2.dpad_down) {wrist_x = arm_x_center;}
                wrist.setPosition(wrist_x);

                // Claw control
                claw.setPower(Input_claw);

                // Hang
                if (gamepad2.y && !arm_busy) {autoHang();}
            }
            else {
                if (gamepad2.b && !arm_busy) {armDeploy();} // Press to deploy arm
            }

            // Speed multiplier. If right trigger is more than half way pressed half the speed output
            double speed_coefficient;
            if (trigger > 0.5) {speed_coefficient = 0.5;} else {speed_coefficient = 1;}

            // Brake Control
            // If engaged the wheels will be locked in place, if not the wheels can be moved freely.
            if (gamepad1.a) {
                brake = true;
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else if (gamepad1.b) {
                brake = false;
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);

            // Basic Telemetry
            telemetry.addData("Status", runtime.toString());
            telemetry.addData("Loop time (ms)", loop_time.milliseconds());
            // Brake Telemetry
            if (brake)  {telemetry.addData("Brake", "Engaged");}
            else        {telemetry.addData("Brake", "Disengaged");}
            telemetry.addData("Speed Multiplier", speed_coefficient);
            //
            telemetry.addData("-- Scoring Device --", "");
            if (arm_active) {
                telemetry.addData("Arm Power", arm_y);
                telemetry.addData("Wrist Position", wrist_x);
            } else {
                telemetry.addData("Arm", "LOCKED");
            }

            // Misc
            telemetry.addData("-- Misc --", ""); //Divider
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();

            loop_time.reset();
        }
    }
}
