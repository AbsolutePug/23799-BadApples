package org.firstinspires.ftc.teamcode.driver.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This stores all the scheduled custom functions used in TeleOp and Basic Autonomous
 */
public class driveFunction {

    // Drive motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor arm;
    private CRServo claw;
    private Servo wrist;
    private boolean robotActive = true; // Whether the bot can be operated by controls
    private boolean armActive = false;
    private boolean brakes = false;

    public final double arm_x_right = 0.18; // Servo arm position for right
    public final double arm_x_center = 0.5; // Servo arm position for center
    public final double arm_x_left = 0.82; // Servo arm position for left

    public driveFunction(DcMotor driveLF, DcMotor driveRF, DcMotor driveLB, DcMotor driveRB, DcMotor arm, Servo wrist, CRServo claw) {
        leftFront = driveLF;
        rightFront = driveRF;
        leftBack = driveLB;
        rightBack = driveRB;
        this.arm = arm;
        this.claw = claw;
        this.wrist = wrist;
        wrist.scaleRange(0,1);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double FL, double FR, double BL, double BR) {
        if (robotActive) {
            leftFront.setPower(FL);
            rightFront.setPower(FR);
            leftBack.setPower(BL);
            rightBack.setPower(BR);
        }
    } // Set the power of each motor
    public void autoHang() {
        ElapsedTime hang_timeout = new ElapsedTime();
        hang_timeout.reset();

        while (robotActive) {
            // Drive
            if (hang_timeout.milliseconds() > 1500 && hang_timeout.milliseconds() < 2500) {
                setPower(0.5,0.5,0.5,0.5);
            }

            // Arm
            if (hang_timeout.milliseconds() < 1600) {
                arm.setPower(1);
            } else {
                arm.setPower(-1);
            }
        }
    } // Completes the ascent procedure
    public void armDeploy() {
        ElapsedTime deploy_timeout = new ElapsedTime();
        deploy_timeout.reset();

        while (deploy_timeout.milliseconds() < 1000) {
            // Arm
            if (deploy_timeout.milliseconds() < 500) {
                claw.setPower(-1);
                arm.setPower(1);
            } // Move up for first 500ms
            else {
                arm.setPower(-1);
            } // Move down for 500ms after 500ms

            // Servo
            if (deploy_timeout.milliseconds() > 500)  {

                wrist.setPosition(arm_x_center);
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } // Move servo after moving arm up
        }
        // Arm has finished changing state
        armActive = true;
        setWrist(arm_x_center);
    } // Deploy arm into ready position


    public void setBrakes(boolean setActive) {
        brakes = setActive;
        if (setActive) {
            leftFront   .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack    .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack   .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            leftFront   .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightFront  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftBack    .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightBack   .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
    public void setBrakes() {
        setBrakes(!brakes);
    }
    public boolean getBrakes() {
        return brakes;
    }

    public boolean getArm() {
        return armActive;
    }
    public void setArmPower(double power) {
        if (robotActive&&armActive){
            arm.setPower(power);
        }
    }

    public void setWrist(double x) {
        if(robotActive&&armActive) {
            wrist.setPosition(x);
        }
    }

    public void setClaw(double power) {
        if(robotActive&&armActive) {
            claw.setPower(power);
        }
    }
}
