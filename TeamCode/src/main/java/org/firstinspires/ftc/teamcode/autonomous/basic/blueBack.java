package org.firstinspires.ftc.teamcode.autonomous.basic;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue Back", group="Custom Blue")
public class blueBack extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    private ElapsedTime runtime = new ElapsedTime();

    // the variables
    long fWait = 0;
    // the methods
    void MotorReset() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
    void Forward(double speed, long  time) {
        telemetry.addData("Status","Forward @ " + speed*100 + "% for " + time);
        telemetry.update();
        runtime.reset();
        fWait = time; //Tell code that started the method how long to wait
        //Set motors
        for (double accel = speed/10; accel < 1; accel = accel + .1) {
            leftFront.setPower(-speed * accel);
            leftBack.setPower(-speed * accel);
            rightFront.setPower(-speed * accel);
            rightBack.setPower(-speed * accel);
            sleep(time / 10);
        }
        MotorReset(); // Reset Motors
    }
    /*Useless turn code!! AAAAa void Turn(double speed, long degrees) {
        telemetry.addData("Status","Turning @ " + speed*100 + "% for " + time);
        telemetry.update();
        leftFront.setPower(speed);
        leftBack.setPower(-speed);
        rightFront.setPower(-speed);
        rightBack.setPower(speed);
        fWait = (long) (degrees / 90 / speed);
        sleep(fWait);
        MotorReset();
    } */
    void Strafe(double speed, long time, boolean right) {
        telemetry.addData("Status","Strafing");
        telemetry.update();
        fWait = time;
        double accel = speed/10;

        // set direction
        int dir;
        if (right){
            dir = 1;
        }
        else {
            dir = -1;
        }

        for (int i = 0; i < 10; i++) {
            accel = accel + .2;
            if (accel > 1) {
                accel = 1;
            }
            leftFront.setPower(speed*dir  *  accel);
            leftBack.setPower(-speed*dir  *  accel);
            rightFront.setPower(-speed*dir  *  accel);
            rightBack.setPower(speed*dir  *  accel);
            sleep(time / 10);
        }

        MotorReset();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the drive system variables.
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        //Brake
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        Forward(1, 400);
        sleep(fWait);
        Strafe(1,500, true);
        sleep(fWait);
        Forward(1,750);
        sleep(fWait);
        Strafe(1,1500,false);
        leftFront.setPower(1);
        leftBack.setPower(1);
        rightFront.setPower(-1);
        rightBack.setPower(-1);
        sleep(100);
        Strafe(1,775,false);
        sleep(fWait);
    }
}