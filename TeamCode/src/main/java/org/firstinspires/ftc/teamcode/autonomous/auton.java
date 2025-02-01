
package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Autonomous", group = "!Main")
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
    //enum SIDE {LEFT,RIGHT,UNDEFINED} Not necessary because this season's field is mirrored
    //SIDE starting_pos = SIDE.UNDEFINED;
    boolean wait = false;


    void dtSetPower(double FL, double FR, double BL, double BR) {
        leftFront.setPower(-FL);
        rightFront.setPower(-FR);
        leftBack.setPower(-BL);
        rightBack.setPower(-BR);
    } // "dt" : Drivetrain // Deprecated

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

        telemetry.addData("WAIT", "NO");
        telemetry.update();
        // SIDE

        while (!isStopRequested() && !isStarted()) {
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                telemetry.addData("WAIT", "NO");
                telemetry.update();
                wait = false;
            }
            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                telemetry.addData("WAIT", "YES");
                telemetry.update();
                wait = true;
            }
        }

        waitForStart();
        runtime.reset();
        claw.setPower(-1);

        if (wait) {
            sleep(10000);
        }

        // Move
        arm.setTargetPosition(5250);
        wrist.setPosition(0.18);
        sleep(1000);
        move(-0.1,1200);

        // Move back
        sleep(200);
        move(1,100);
        claw.setPower(0);
        arm.setTargetPosition(0);
        move(1,100);


        // Strafe
        moveAdvanced(-0.1, 0.1, 0.1, -0.1, 3000);
        sleep(100);
        move(-0.1,200);
    }   // end runOpMode()
}   // end class