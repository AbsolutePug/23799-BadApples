package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
    private DcMotorEx flywheel_1;
    private DcMotorEx flywheel_2;
    // Scoring Device
    private DcMotor intake;
    private Servo blocker;
    private CRServo feeder_1;
    private CRServo feeder_2;
    // Extra
    private boolean robot_active = true; // Whether the bot can be operated by controls, used for some automatic override
    private boolean flywheel_active = false;
    public enum Brake {
        ENGAGED,
        DISENGAGED
    } // Valid brake states
    public Brake brake_state = Brake.DISENGAGED; // This is the default state of any motor
    ElapsedTime flywheel_time = new ElapsedTime(); // How long the flywheel has been active, used to determine if the motor has reached top speed

    public final double LAUNCHER_TARGET_VELOCITY = 1350; //Target velocity for far goal
    public final double LAUNCHER_MIN_VELOCITY = 1325; // Minimum velocity to shoot

    /**
     * Initialize the RobotHardware by using the ROBOT's {@link com.qualcomm.robotcore.hardware.HardwareMap}
     * @param hwMap ROBOT's {@link com.qualcomm.robotcore.hardware.HardwareMap} for the OpMode
     */
    public void init(HardwareMap hwMap) {
        // Assign motor variables
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        leftBack = hwMap.get(DcMotor.class, "leftBack");
        rightBack = hwMap.get(DcMotor.class, "rightBack");
        flywheel_1= hwMap.get(DcMotorEx.class, "launcher1");
        flywheel_2 = hwMap.get(DcMotorEx.class, "launcher2");
        intake = hwMap.get(DcMotor.class, "intake");
        blocker = hwMap.get(Servo.class, "blocker");
        feeder_1 = hwMap.get(CRServo.class, "guider_1");
        feeder_2 = hwMap.get(CRServo.class, "guider_2");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Init drive motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        flywheel_1.setDirection(DcMotor.Direction.REVERSE);
        flywheel_2.setDirection(DcMotor.Direction.REVERSE);
        flywheel_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        feeder_1.setDirection(CRServo.Direction.FORWARD);
        feeder_2.setDirection(CRServo.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Set each wheel's power
     * @param FL Front Left power
     * @param FR Front Right power
     * @param BL Back left power
     * @param BR Back right power
     */
    public void setPowers(double FL, double FR, double BL, double BR) {
        if (robot_active) {
            leftFront.setPower(FL);
            rightFront.setPower(FR);
            leftBack.setPower(BL);
            rightBack.setPower(BR);
        }
    } // Set the power of each motor

    // Brakes
    public void setBrakes(Brake setState) {
        brake_state = Brake.DISENGAGED;
        switch (setState) {
            case ENGAGED:
                leftFront   .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFront  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftBack    .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);rightBack   .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                brake_state = Brake.ENGAGED;
                break;
            case DISENGAGED:
                leftFront   .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightFront  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftBack    .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightBack   .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                break;
            default: // fallback
                leftFront   .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightFront  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftBack    .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightBack   .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                brake_state = Brake.DISENGAGED;
                break;
        }
    }
    /**
     * @return Return the brakes enum state as a boolean
     */
    public boolean getBrakes() {
        return (brake_state == Brake.ENGAGED);
    }
    // Flywheel
    public void setFlywheel(boolean isOn) {
        if (isOn) {
            flywheel_1.setVelocity(LAUNCHER_TARGET_VELOCITY);
            flywheel_2.setVelocity(-LAUNCHER_TARGET_VELOCITY);
            flywheel_time.reset();
            flywheel_active = true;
        } else {
            flywheel_1.setVelocity(0);
            flywheel_2.setVelocity(0);
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
        return ((flywheel_1.getVelocity() >= LAUNCHER_MIN_VELOCITY) && (flywheel_2.getVelocity() >= LAUNCHER_MIN_VELOCITY));
    }
    public double getFlywheelVelocity() {
        return Math.min(flywheel_1.getVelocity(), flywheel_2.getVelocity());
    }
    public double getFlywheelMaxVelocity() {
        return LAUNCHER_MIN_VELOCITY;
    } // i think this could be static
    // Intake
    public void setIntake(boolean state) {
        if (state) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
    }
    public void setIntake(double power) {
        intake.setPower(power);
    }
    // Feeder
    public void setFeeder(boolean state) {
        if (state) {
            feeder_1.setPower(1);
            feeder_2.setPower(1);
        } else {
            feeder_1.setPower(0);
            feeder_2.setPower(0);
        }
    }
    public void setFeeder(double power) {
        feeder_1.setPower(power);
        feeder_2.setPower(power);
    }
    // Blocker
    public void setBlocker(double pos) {
        blocker.setPosition(pos);
    }
    public void setBlocker(boolean state) { // unfinished
        if (state) {
            blocker.setPosition(0);
        }
    }
}
