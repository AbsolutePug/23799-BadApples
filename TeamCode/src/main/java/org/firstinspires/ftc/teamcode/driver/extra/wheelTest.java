package org.firstinspires.ftc.teamcode.driver.extra;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// This is a simple script to test the drive motors on the robot. This is used to tell if motor directions are set correctly.


/**
 * Written by <a href="https://github.com/AbsolutePug">Robert Maddox (AbsolutePug)</a> 2025
 * Simple OpMode to determine the correct drive motor directions
 */
@TeleOp(name="Motor Tester", group="Test")
public class wheelTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        //Motor Direction
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Inputs
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean right = gamepad1.dpad_right;
            double power = .25;

            if (up) {
                leftFront.setPower(-power);
                rightFront.setPower(-power);
                leftBack.setPower(-power);
                rightBack.setPower(-power);
            }
            if (down) {
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
            }
            if (right) {
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
            }
            telemetry.addData("Status", "Running forward");
        }
    }}
