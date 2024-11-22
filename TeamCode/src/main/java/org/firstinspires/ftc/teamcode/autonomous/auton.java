
package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Autonomous", group = "Autonomous")
public class auton extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    //Motors
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor arm = null;
    private Servo wrist = null;
    private CRServo claw = null;

    // Variables
    // Encoder
    final double wheel_diameter = 9.6; // cm
    final double circumference  = Math.PI * wheel_diameter;
    final double resolution     = 537.7;
    final double to_rotation    = 1/circumference; // rotations per cm

    String side = "LEFT";
    boolean back = false;

    void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void dtSetPower(double FL, double FR, double BL, double BR) {
        leftFront.setPower(-FL);
        rightFront.setPower(-FR);
        leftBack.setPower(-BL);
        rightBack.setPower(-BR);
    } // "dt" : Drivetrain

    void move(double FL, double FR, double BL, double BR, double time) {
        ElapsedTime timeout = new ElapsedTime();
        timeout.reset();
        while (!isStopRequested() && timeout.milliseconds() < time) {
            leftFront.setPower(-1*FL);
            rightFront.setPower(-1*FR);
            leftBack.setPower(-1*BL);
            rightBack.setPower(-1*BR);
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
    void Linear(double dist) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double num_points = dist*to_rotation*resolution; // Multiply distance in centimeters to convert to rotations, then convert to the encoder's resolution
        while (leftFront.getCurrentPosition() < num_points && !isStopRequested()) {
            leftFront.setPower(-.3);
            rightFront.setPower(-.3);
            leftBack.setPower(-.3);
            rightBack.setPower(-.3);
            telemetry.addData("DIST",leftFront.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    @Override
    public void runOpMode(){
        // Initialize the hardware variables.
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        arm = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(Servo.class, "wrist");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Side", "NOT SET");
        telemetry.addData("Ascent Zone","NOT SET");
        telemetry.update();

        while (!isStopRequested() && !isStarted()) {
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                telemetry.addData("Side", "LEFT");
                telemetry.update();
                side = "LEFT";
            }
            if (gamepad1.dpad_right || gamepad2.dpad_right) {
                telemetry.addData("Side", "RIGHT");
                telemetry.update();
                side = "RIGHT";
            }

            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                telemetry.addData("Ascent Zone", "FRONT");
                telemetry.update();
                back = false;
            }
            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                telemetry.addData("Ascent Zone", "BACK");
                telemetry.update();
                back = true;
            }
        }


        waitForStart();
        runtime.reset();

        Linear(30);
    }   // end runOpMode()
}   // end class