package org.firstinspires.ftc.teamcode.driver.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;


/**
 * This stores all the scheduled custom functions used in TeleOp and Basic Autonomous
 */
public class driveFunction {

    // Drive motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor ballLauncher;
    private CRServo mover1;
    private CRServo mover2;
    private boolean robot_active = true; // Whether the bot can be operated by controls
    private boolean launcher_active = false;
    private boolean brakes = false; // Whether the robot is set to be false: motors will coast, or true: robot wheels will be locked with zero power

    public final double arm_x_right = 0.18; // Servo arm position for right
    public final double arm_x_center = 0.5; // Servo arm position for center
    public final double arm_x_left = 0.82; // Servo arm position for left

    public driveFunction(DcMotor driveLF, DcMotor driveRF, DcMotor driveLB, DcMotor driveRB, DcMotor launcher, CRServo mover1, CRServo mover2) {
        leftFront = driveLF;
        rightFront = driveRF;
        leftBack = driveLB;
        rightBack = driveRB;
        ballLauncher = launcher;
        this.mover1 = mover1;
        this.mover2 = mover2;

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        ballLauncher.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double FL, double FR, double BL, double BR) {
        if (robot_active) {
            leftFront.setPower(FL);
            rightFront.setPower(FR);
            leftBack.setPower(BL);
            rightBack.setPower(BR);
        }
    } // Set the power of each motor


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

    public void setLauncher(boolean isOn) {
        if (isOn) {
            ballLauncher.setPower(0.75);
            launcher_active = true;
        } else {
            ballLauncher.setPower(0);
            launcher_active = false;
        }
    }
    public void setLauncher() {
        setLauncher(!launcher_active);
    }
    public boolean getLauncher() {
        return launcher_active;
    }

    public void setMover(boolean isOn) {
        if (isOn) {
            mover1.setPower(1);
            mover2.setPower(-1);
        } else {
            mover1.setPower(0);
            mover2.setPower(0);
        }
    }
}
