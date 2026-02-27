package org.firstinspires.ftc.teamcode.BadApples.Core;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


/**
 * Written by <a href="https://github.com/AbsolutePug">AbsolutePug</a> 2025
 * <p>
 * This stores all the ROBOT's possible functions for utilization on the TeleOp and Autonomous
 */
public class RobotFunctionCore {

    // Drive motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    // Scoring Device
    private DcMotor intake;
    private Servo channel;
    private CRServo launcher_right;
    private CRServo launcher_left;
    private DcMotorEx flywheel_1;
    private DcMotorEx flywheel_2;
    private int flywheel_1_velocity = 0;
    private int flywheel_2_velocity = 0; // TODO: use one method call to update velocities, decreases loop time
    private FlywheelSpeed flywheel_state = FlywheelSpeed.OFF;
    public enum FlywheelSpeed {
        OFF,
        SHORT,
        FAR
    } // Flywheel target velocities
    public enum Intake {
        OFF,
        ON,
        REBUKE
    } // Intake settings
    // Extra
    private boolean robot_active = true; // Whether the bot can be operated by controls, used for some automatic override
    public enum Brake {
        ENGAGED,
        DISENGAGED
    } // Valid brake states
    public Brake brake_state = Brake.DISENGAGED; // This is the default state of any motor

    // Sensors
    public final double FLYWHEEL_TARGET_VELOCITY_FAR = 1400; // Target velocity for far goal
    public final double FLYWHEEL_TARGET_VELOCITY_SHORT = 1050; // Target velocity for far goal
    public double FLYWHEEL_1_VELOCITY = 0;
    public double FLYWHEEL_2_VELOCITY = 0;
    public FtcDashboard dashboard;

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
        channel = hwMap.get(Servo.class, "blocker");
        launcher_right = hwMap.get(CRServo.class, "guider_1");
        launcher_left = hwMap.get(CRServo.class, "guider_2");

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

        launcher_right.setDirection(CRServo.Direction.FORWARD);
        launcher_left.setDirection(CRServo.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        dashboard = FtcDashboard.getInstance();
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
                leftBack    .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBack   .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
    public void setFlywheel(FlywheelSpeed new_state) {
        flywheel_state = new_state;
        switch (new_state) {
            case FAR:
                setFlywheel(FLYWHEEL_TARGET_VELOCITY_FAR);
                break;
            case SHORT:
                setFlywheel(FLYWHEEL_TARGET_VELOCITY_SHORT);
                break;
            case OFF:
                setFlywheel(0);
                break;
        }
    }
    public void setFlywheel(double target_velocity) {
        flywheel_1.setVelocity(target_velocity);
        flywheel_2.setVelocity(-target_velocity);
    }
    public void setFlywheel() {
        setFlywheel(FlywheelSpeed.OFF);
    }
    public FlywheelSpeed getFlywheel() {
        return flywheel_state;
    }
    public boolean getFlywheelReady() {
        return (getFlywheelVelocity() < getFlywheelMinVelocity()+50 && getFlywheelVelocity() > getFlywheelMinVelocity()-50);
    }
    public double getFlywheelReadyAsDecimal() {
        return (FLYWHEEL_1_VELOCITY+FLYWHEEL_2_VELOCITY)/2 / getFlywheelMinVelocity();
    }
    public double getFlywheelVelocity() {
        return (Math.abs(FLYWHEEL_1_VELOCITY) + Math.abs(FLYWHEEL_2_VELOCITY))/2;
    }
    public double getFlywheelMinVelocity() {
        switch (flywheel_state) {
            case FAR:
                return  FLYWHEEL_TARGET_VELOCITY_FAR;
            case SHORT:
                return  FLYWHEEL_TARGET_VELOCITY_SHORT;
        }
        return 0;
    }
    // Intake
    public void setIntake(boolean state) {
        if (state) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
    }
    public void setIntake(Intake new_state) {
        switch (new_state) {
            case ON:
                intake.setPower(1);
                break;
            case REBUKE:
                intake.setPower(-1);
                break;
            default:
                intake.setPower(0);
                break;
        }
    }
    public void setIntake(double power) {
    }
    // Launcher
    public void setLauncher(boolean state) {
        if (state) {
            launcher_right.setPower(1);
            launcher_left.setPower(1);
        } else {
            launcher_right.setPower(0);
            launcher_left.setPower(0);
        }
    }
    public void setLauncher(boolean state_left, boolean state_right) {
        if (state_left) {
            launcher_left.setPower(1);
        } else {
            launcher_left.setPower(0);
        }
        if (state_right) {
            launcher_right.setPower(1);
        } else {
            launcher_right.setPower(0);
        }
    }
    public void setLauncher(double power) {
        launcher_right.setPower(power);
        launcher_left.setPower(power);
    }
    // Blocker NOTE: I think we arent using this anymore
    public void setChannel(double pos) {
        channel.setPosition(pos);
    }
    public void setBlocker(boolean state) { // unfinished
        if (state) {
            channel.setPosition(0);
        }
    }
    // Sensors
    public void updateSensors() {
        FLYWHEEL_1_VELOCITY = flywheel_1.getVelocity();
        FLYWHEEL_2_VELOCITY = flywheel_2.getVelocity();
    }
}