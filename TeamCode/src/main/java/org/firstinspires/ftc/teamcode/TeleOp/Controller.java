package org.firstinspires.ftc.teamcode.TeleOp;
//import com.acmerobotics.roadrunner.Math;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp", group="Custom")
public class Controller extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    private Servo Drone = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        Drone = hardwareMap.get(Servo.class, "drone");



        // Motor Direction
        Drone.setDirection(Servo.Direction.FORWARD);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

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

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // Controller Inputs
            double axial = gamepad1.left_stick_y; // Forward
            double lateral = gamepad1.left_stick_x; // Strafe
            double yaw = -gamepad1.right_stick_x; // Turn
            double trigger = gamepad1.right_trigger; // Right Trigger (Slowmode)
            boolean brakeEng = gamepad1.a; // Brake Control
            boolean brakeDis = gamepad1.b; // Brake Disengage
            boolean dStatus = false; //Status of drone launcher as launched or not. Used for telemetry
            boolean dLaunch = (gamepad1.right_bumper  || gamepad2.right_bumper); // Launch Drone
            boolean dReset = (gamepad1.left_bumper || gamepad2.left_bumper); // Reset Drone Servo

            // Accuracy mode. If right stick is more than half way pressed. Enable Accuracy mode (slowmode)
            boolean accMode = trigger > .5;

            // Brake Control
            if (brakeEng) { // If brake engage button is pressed
                brake = true;
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } // If brake engage button is pressed
            if (brakeDis) {
                brake = false;
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            } // If brake disengage button is pressed

            // Drone Launcher Control
            double dtLaunch = .25; // Target position for servo when launched
            double dtReset = 0; // Target position for servo when reset
            if (dLaunch) {
                Drone.setPosition(dtLaunch);
                dStatus = true;
            } // If drone launch button is pressed
            else if (dReset) {
                dStatus = false;
                Drone.setPosition(dtReset);
            } // If drone servo reset button is pressed

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
            if (accMode) {
                leftFrontPower = leftFrontPower * .5;
                rightFrontPower = rightFrontPower * .5;
                leftBackPower = leftBackPower * .5;
                rightBackPower = rightBackPower * .5;
            } //If accuracy mode is enabled half the set power

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
            telemetry.addData("Status", "Run Time: " + runtime);
            // Brake Telemetry
            if (brake)  {telemetry.addData("Brake", "Engaged");}
            else        {telemetry.addData("Brake", "Disengaged");}
            telemetry.addData("Accuracy Mode", accMode);

            // Drone Launcher Telemetry
            telemetry.addData("-- Drone --", ""); //Divider
            telemetry.addData("Drone Status", dStatus);
            telemetry.addData("Drone Position", Drone.getPosition());

            // Misc

            telemetry.addData("-- Misc --", ""); //Divider
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();

        }
    }
}
