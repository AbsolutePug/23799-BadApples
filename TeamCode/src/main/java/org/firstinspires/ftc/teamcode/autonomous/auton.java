
package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

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

    // Constants
    final double arm_x_right = 0.18;
    final double arm_x_center = 0.5;
    final double arm_x_left = 0.82;

    // Variables
    double invert = 1; // Variable to invert power if on the other side
    double wrist_x = arm_x_left;

    void armDeploy() {
        ElapsedTime deploy_timeout = new ElapsedTime();
        deploy_timeout.reset();

        while (deploy_timeout.milliseconds() < 1000 && !isStopRequested()) {
            // Beginning
            if (deploy_timeout.milliseconds() < 600) {
                arm.setPower(1);
            } // Move up for first 500ms
            else {
                arm.setPower(-1);
            } // Move down for 500ms after 500ms
            // Servo
            if (deploy_timeout.milliseconds() > 500)  {
                wrist.setPosition(arm_x_center);
                wrist_x = arm_x_center;
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } // Move servo after moving arm up
        }
        // Arm has finished changing state
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
        while (!isStopRequested()) {
            if (timeout.milliseconds() < time) {
                leftFront.setPower(-FL);
                rightFront.setPower(-FR);
                leftBack.setPower(-BL);
                rightBack.setPower(-BR);
            }
            else {
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                break;
            }
        }

    }

    void armSet(double Power) {

    }
    void autoHang() {
        ElapsedTime hang_timeout = new ElapsedTime();
        hang_timeout.reset();

        while (opModeIsActive()) {
            // Drive
            if (hang_timeout.milliseconds() > 1500 && hang_timeout.milliseconds() < 2500) {
                dtSetPower(0.5,0.5,0.5,0.5);
            } else {
                dtSetPower(0,0,0,0);
            }

            // Arm
            if (hang_timeout.milliseconds() < 1600) {
                arm.setPower(1);
            } else {
                arm.setPower(-1);
            }
        }
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
        wrist.scaleRange(0,1);
        claw = hardwareMap.get(CRServo.class, "claw");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Side", "SIDE NOT SET");
        telemetry.update();

        while (!isStopRequested()) {
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                telemetry.addData("Side", "LEFT");
                invert = 1;
                break;
            }
            if (gamepad1.dpad_right || gamepad2.dpad_right) {
                telemetry.addData("Side", "RIGHT");
                invert = -1;
                break;
            }
        }
        telemetry.update();

        waitForStart();
        runtime.reset();

        armDeploy();
        move(-1*invert,1*invert,1*invert,-1*invert,1500); // Strafe right if on left side, strafe left if on right side
        move(-1,-1,-1,-1,1000);
        autoHang();
    }   // end runOpMode()
}   // end class