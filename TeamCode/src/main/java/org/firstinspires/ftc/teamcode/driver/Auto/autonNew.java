
package org.firstinspires.ftc.teamcode.driver.Auto;

import static com.acmerobotics.roadrunner.ftc.OTOSKt.RRPoseToOTOSPose;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "auto new", group = "!Main")
public class autonNew extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    //Motors
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor arm = null;
    private Servo wrist = null;
    private CRServo claw = null;
    private SparkFunOTOSCorrected otos = null;

    // Variables
    //enum SIDE {LEFT,RIGHT,UNDEFINED} Not necessary because this season's field is mirrored
    //SIDE starting_pos = SIDE.UNDEFINED;
    boolean wait = false;


    void dtSetPower(double FL, double FR, double BL, double BR) {
        leftFront.setPower(-FL);
        rightFront.setPower(-FR);
        leftBack.setPower(-BL);
        rightBack.setPower(-BR);
    } // "dt" : Drivetrain // Deprecated
    void Translate(double TargetX, double TargetY) {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        otos.setPosition(RRPoseToOTOSPose(beginPose));
        double lateral = 0;
        double axial = 0;
        while (!isStopRequested()) {
            pose2d = otos.getPosition();
            double x = pose2d.x;
            double y = pose2d.y;
            telemetry.addData("x", x);
            telemetry.addData("y", y);

            // Error in X and Y
            double errorX = TargetX - x;
            double errorY = TargetY - y;

            // Proportional Control
            double kP = 0.5; // Tune this value to adjust responsiveness

            axial   = kP * errorY;  // Move forward/backward based on Y error
            lateral = kP * errorX;  // Move left/right based on X error

            // Ensure motors don't exceed power limits
            double maxPower = 0.25;
            leftFront.setPower (Math.max(-maxPower, Math.min(axial - lateral, maxPower)));
            rightFront.setPower(Math.max(-maxPower, Math.min(axial + lateral, maxPower)));
            leftBack.setPower  (Math.max(-maxPower, Math.min(axial + lateral, maxPower)));
            rightBack.setPower (Math.max(-maxPower, Math.min(axial - lateral, maxPower)));

            telemetry.update();
        }
    }

    void moveAdvanced(double FL, double FR, double BL, double BR, double time) {
        ElapsedTime timeout = new ElapsedTime();
        ElapsedTime accel = new ElapsedTime();
        timeout.reset();
        accel.reset();

        while (!isStopRequested() && timeout.milliseconds() < time) {
            leftFront.setPower(-1*FL);
            rightFront.setPower(-1*FR);
            leftBack.setPower(-1*BL);
            rightBack.setPower(-1*BR);
            telemetry.addData("timeout %",timeout.milliseconds()*100 / time);
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
    void move(double speed, double time) {
        ElapsedTime timeout = new ElapsedTime();
        timeout.reset();

        while (!isStopRequested() && timeout.milliseconds() < time) {
            leftFront.setPower(-speed);
            rightFront.setPower(-speed);
            leftBack.setPower(-speed);
            rightBack.setPower(-speed);
            telemetry.addData("timeout %",timeout.milliseconds()*100 / time);
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
    void setArm(double position) {

    }

    SparkFunOTOS.Pose2D pose2d = new SparkFunOTOS.Pose2D();

    @Override
    public void runOpMode(){
        // Initialize the hardware variables.
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        arm = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(CRServo.class, "claw");
        otos = hardwareMap.get(SparkFunOTOSCorrected.class,"sensor_otos");

        wrist.scaleRange(0,1);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        otos.calibrateImu(255, false);

        waitForStart();
        runtime.reset();
        Translate(1,0);

    }   // end runOpMode()
}   // end class