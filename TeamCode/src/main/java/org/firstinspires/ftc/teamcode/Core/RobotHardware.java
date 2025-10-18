package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Written by <a href="https://github.com/AbsolutePug">Robert Maddox (AbsolutePug)</a> 2025
 * <p>
 * This stores all the ROBOT's possible functions for utilization on the TeleOp and Autonomous
 */
public class RobotHardware {

    // Drive motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor flywheel;
    private CRServo guider1;
    private CRServo guider2;
    private boolean robot_active = true; // Whether the bot can be operated by controls, used for some automatic override
    private boolean flywheel_active = false;
    private boolean brakes = false; // Whether the brakes is set to be false: motors will coast, or true: robot wheels will be locked with zero power
    ElapsedTime flywheel_time = new ElapsedTime(); // How long the flywheel has been active, used to determine if the motor has reached top speed

    public double front_left_dir = 1;
    public double front_right_dir = -1;
    public double back_left_dir = 1;
    public double back_right_dir = -1;

    /**
     * Initialize the RobotHardware by using the ROBOT's {@link com.qualcomm.robotcore.hardware.HardwareMap}
     * @param hwMap ROBOT's {@link com.qualcomm.robotcore.hardware.HardwareMap} for the OpMode
     */
    public void init(HardwareMap hwMap) {
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        leftBack = hwMap.get(DcMotor.class, "leftBack");
        rightBack = hwMap.get(DcMotor.class, "rightBack");
        flywheel = hwMap.get(DcMotor.class, "launcher");
        this.guider1 = hwMap.get(CRServo.class, "mover1");
        this.guider2 = hwMap.get(CRServo.class, "mover2");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel.setDirection(DcMotor.Direction.REVERSE);
    }
    /**
     * Set each wheel's power
     * @param FL Front Left power
     * @param FR Front Right power
     * @param BL Back left power
     * @param BR Back right power
     */
    public void setPower(double FL, double FR, double BL, double BR) {
        if (robot_active) {
            leftFront.setPower(FL*front_left_dir);
            rightFront.setPower(FR*front_right_dir);
            leftBack.setPower(BL*back_left_dir);
            rightBack.setPower(BR*back_right_dir);
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
    /**
     * @return Brake state
     */
    public boolean getBrakes() {
        return brakes;
    }

    public void setFlywheel(boolean isOn) {
        if (isOn) {
            flywheel.setPower(0.75);
            flywheel_active = true;
            flywheel_time.reset();
        } else {
            flywheel.setPower(0);
            flywheel_active = false;
        }
    }
    public void setFlywheel() {
        setFlywheel(!getFlywheel());
    }
    public boolean getFlywheel() {
        return flywheel_active;
    }
    public boolean getFlywheelReady() {
        return flywheel_active && (flywheel_time.milliseconds() > 1000);
    } // Flywheel has reached top speed
    public double getFlywheelTime() {
        if (flywheel_active) {
            return flywheel_time.milliseconds();
        } else {
            return 0;
        }

    }

    public void setGuider(boolean isOn) {
        if (isOn) {
            guider1.setPower(1);
            guider2.setPower(-1); // The direction of the second guide wheel is reversed for some reason
        } else {
            guider1.setPower(0);
            guider2.setPower(0);
        }
    }
    public void setGuider(double power) {
        guider1.setPower(power);
        guider2.setPower(-power);
    }
}
