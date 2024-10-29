package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
/*
Written by Robert Maddox (2024) @AbsolutePug (github.com/AbsolutePug)
*/

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Custom")
public class TeleOp extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime loop_time = new ElapsedTime();
    private final ElapsedTime arm_timeout = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor arm = null;
    private Servo wrist = null;
    private CRServo claw = null;

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
        Input_brake = gamepad_a && gamepad_a_last;
        if (gamepad2.right_bumper && !gamepad_right_bumper_last) {
            if (Input_claw == 1) {Input_claw = 0;}
            else                 {Input_claw = 1;}
        }
        if (gamepad2.left_bumper && !gamepad_left_bumper_last) {
            if (Input_claw == -1) {Input_claw = 0;}
            else                  {Input_claw = -1;}
        }

        // Store the input recorded this update
        gamepad_a_last = gamepad_a;
        gamepad_left_bumper_last  = gamepad2.left_bumper;
        gamepad_right_bumper_last = gamepad2.right_bumper;
    }
    void armDeploy() {

    }

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        arm = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.scaleRange(0,1);
        claw = hardwareMap.get(CRServo.class, "claw");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Init, Ready for Start!");
        telemetry.update();

        waitForStart();
        runtime.reset();

        //Set brake to true initially
        boolean brake = true;
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Variables
        boolean arm_active = false;
        boolean arm_state_change = false; // Is the arm currently changing state?
        boolean active = false;
        final double arm_x_right = 0.18;
        final double arm_x_center = 0.5;
        final double arm_x_left = 0.82;
        double wrist_x = arm_x_left;

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
            if (!arm_state_change) {
                if (arm_active) {
                    arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    arm.setPower(arm_y);
                    // Wrist Control
                    if (gamepad2.dpad_left) {wrist_x = arm_x_left;}
                    else if (gamepad2.dpad_right) {wrist_x = arm_x_right;}
                    else if (gamepad2.dpad_up || gamepad2.dpad_down) {wrist_x = arm_x_center;}
                    wrist.setPosition(wrist_x);

                    // Claw control
                    claw.setPower(Input_claw);
                }
                else {
                    if (gamepad2.b) {
                        arm_state_change = true;
                        arm_timeout.reset();
                    } // Press to change state
                    arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    arm.setPower(0);
                    wrist.setPosition(arm_x_right);
                    claw.setPower(0);
                }
            } // Only allow control of arm if it is not changing state
            // Arm is currently changing states
            if (arm_state_change) {
                // Beginning
                if (arm_timeout.milliseconds() < 500) {arm.setPower(1);}
                else                                   {arm.setPower(-1);} // Arm Y Movement
                //
                if (arm_timeout.milliseconds() > 500)  {
                    if (arm_active) {wrist.setPosition(arm_x_right);} // Changing from active to inactive
                    else            {wrist.setPosition(arm_x_center);
                                     wrist_x = arm_x_center;       } // Changing from inactive to active
                    arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                } // Move servo after moving arm up
                //
                if (arm_timeout.milliseconds() > 1000) {
                    arm_active = !arm_active;
                    arm_state_change = false;
                } // Arm has finished changing state
            } // Arm changing


            // Accuracy mode. If right stick is more than half way pressed. Enable Accuracy mode (slow-mode)
            boolean accuracy_mode = trigger > .5;

            // Brake Control
            // If engaged the wheels will be locked in place, if not the wheels can be moved freely.
            if (Input_brake) { // If brake engage button is pressed
                brake = !brake;
                if (brake) {
                    leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                } else {
                    leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
            if (accuracy_mode) {
                leftFrontPower = leftFrontPower * .5;
                rightFrontPower = rightFrontPower * .5;
                leftBackPower = leftBackPower * .5;
                rightBackPower = rightBackPower * .5;
            } // If accuracy mode is enabled half the set slide_power

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
            telemetry.addData("Status", "" + runtime);
            telemetry.addData("Loop time", loop_time.toString());
            // Brake Telemetry
            if (brake)  {telemetry.addData("Brake", "Engaged");}
            else        {telemetry.addData("Brake", "Disengaged");}
            telemetry.addData("Accuracy Mode", accuracy_mode);
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
