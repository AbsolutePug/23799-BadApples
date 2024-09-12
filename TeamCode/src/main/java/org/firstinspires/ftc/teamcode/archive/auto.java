// This code cannot be used as tfod (Tensor Flow Object Detection) has been removed from the FTC SDK as of v1.0.
// The entirety hsa for the time being been commented out to prevent errors.
/*package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "Autonomous", group = "Autonomous Smart")
@Disabled
public class auto extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    //Motors
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;


    //The variable to store our instance of the TensorFlow Object Detection processor.

    private TfodProcessor tfod;
    private static final String[] Labels = {"Blue Apple", "Red Apple",};

    //The variable to store our instance of the vision portal.
    private VisionPortal visionPortal;

    // Variable
    long fWait = 0;
    double x = 0;
    double y = 0;
    double objCount = 0;
    String objLoc = "";
    boolean foundObj = false;

    // Relevant to position
    boolean blue = false; // By default, program will move right. Set this to move left (Team Color)
    boolean frontStage = false;

    boolean partial = false;
    boolean fallback = false;

    // the methods
    void motorReset() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
    void Forward(double speed, long  milliseconds) {
        telemetry.addData("Status","Forward @ " + speed*100 + "% for " + milliseconds);
        telemetry.update();
        runtime.reset();
        //Set motors
        for (double accel = 0; accel < 1; accel = accel + .1) {
            leftFront.setPower(speed * accel);
            leftBack.setPower(speed * accel);
            rightFront.setPower(speed * accel);
            rightBack.setPower(speed * accel);
            sleep(milliseconds / 10);
        }
        motorReset(); // Reset Motors
    }
    void Strafe(double speed, long milliseconds, boolean right) {
        telemetry.addData("Status","Strafing");
        telemetry.update();
        fWait = milliseconds;
        double accel = 0;

        // set direction
        int dir;
        if (right){dir = 1;}    else {dir = -1;}

        for (int i = 0; i < 10; i++) {
            accel = accel + .2;
            if (accel > 1) {
                accel = 1;
            }
            leftFront.setPower(speed*dir  *  accel);
            leftBack.setPower(-speed*dir  *  accel);
            rightFront.setPower(-speed*dir  *  accel);
            rightBack.setPower(speed*dir  *  accel);
            sleep(milliseconds / 10);
        }

        motorReset();
    }
    void tankTurn(double speed, long milliseconds, boolean right) {
        if (right) {
            leftFront.setPower(speed);
            leftBack.setPower(speed);
            rightFront.setPower(-speed);
            rightBack.setPower(-speed);
        }
        else {
            leftFront.setPower(-speed);
            leftBack.setPower(-speed);
            rightFront.setPower(speed);
            rightBack.setPower(speed);
        }
        sleep(milliseconds);
        motorReset();
    }
    void faceObject(double timeout) {
        telemetry.addData("Status", "Turning to face object");
        refresh();
        runtime.reset();
        while (objCount == 0 && !isStopRequested() && runtime.milliseconds() < timeout) {refresh();}
        while (!isStopRequested() && objCount > 0 && x < 310 || x > 330) {
                double power =  (x - 320) / 320 * .5;
                if (power < .1 && power > 0) {
                    power = .1;
                }
                if (power > -.1 && power < 0) {
                    power = -.1;
                }
                leftFront.setPower(power);
                leftBack.setPower(power);
                rightFront.setPower(-power);
                rightBack.setPower(-power);
                refresh();
        }
        motorReset();
        telemetry.addData("Status", "Finished facing object");
        }
    void refresh() {
        telemetry.addData("Runtime",runtime.seconds());
        List<Recognition> currentRecognitions = tfod.getRecognitions(); // Map Recognitions to list
        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop
        objCount = currentRecognitions.size();
        telemetry.addData("# Objects Detected", objCount);
        telemetry.update();
    }

    @Override
    public void runOpMode(){
        // Define Motors
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initTFOD();

        // Set Positions
        while (!isStopRequested()) {
            telemetry.addData("For RED, BACKSTAGE", "A");
            telemetry.addData("For RED, FRONT STAGE", "B");
            telemetry.addData("For BLUE, BACKSTAGE", "X");
            telemetry.addData("For BLUE, FRONT STAGE", "Y");
            telemetry.addData("PARTIAL, RIGHT BUMPER", partial);
            telemetry.addData("FALLBACK, LEFT BUMPER", fallback);


            if (gamepad1.a) {
                blue = false;
                frontStage = false;
                break;
            } // Red Back
            if (gamepad1.b) {
                blue = false;
                frontStage = true;
                break;
            } // Red, Front
            if (gamepad1.x) {
                blue = true;
                frontStage = false;
                break;
            } // Blue, Back
            if (gamepad1.y) {
                blue = true;
                frontStage = true;
                break;
            } // Blue, Front

            if (gamepad1.right_bumper)
            {
                partial = true;
            }

            telemetry.update();
        } // Wait until specified
        telemetry.clearAll();

        // Initialized
        telemetry.addData("Status", "Initialized");
        if (!blue) {telemetry.addData("Color", "Red");}
        else {telemetry.addData("Color", "Blue");}
        if (frontStage) {telemetry.addData("Position", "Front Stage");}
        else {telemetry.addData("Position", "Back Stage");}
        telemetry.addData("Partial", partial);
        telemetry.update();




        waitForStart();
        runtime.reset();

        // Wait until object is seen
        while (objCount == 0 && !isStopRequested() && runtime.milliseconds() < 5000) {refresh();}

        foundObj = objCount > 0; // Set to true if there are 1 or more objects found
        telemetry.addData("Found Object", foundObj);
        telemetry.update();

        // Store TFOD data
        if (foundObj) {
            if (x > 500) {
                objLoc = "R";
                telemetry.addData("Object Location", "Right");
                telemetry.update();
            } // object is right
            else if (x < 500) {
                objLoc = "C";
                telemetry.addData("Object Location", "Center");
                telemetry.update();
            } // object is center
            else {
                objLoc = "L";
                telemetry.addData("Object Location", "Left");
                telemetry.update();
            } // object is left
        } // Obj is seen
        else {objLoc = "L";} // Obj is not seen

        // Take action

        // Is the object center?
        if (objLoc.equals("C")) {
            Forward(.5, 2000);
            sleep(100);
            Forward(-1, 300);
            sleep(250);
            if (frontStage && !partial) {
                Strafe(1,500, !blue);
                sleep(100);
                Forward(.5,2400);
                sleep(100);
                Strafe(.5, 3750, blue);
                Strafe(1, 900 , blue);
            }
            if (!frontStage && !partial) {
                Forward(-1,300);
                Strafe(.25,5000, blue);
            }
        }

        // Is the object away far from truss?
        if ((frontStage && !blue && objLoc.equals("L")) ||
            (frontStage && blue && objLoc.equals("R")) ||
            (!frontStage && !blue && objLoc.equals("R")) ||
            (!frontStage && blue && objLoc.equals("L")))
        {
            Strafe(1,500, !blue);
            Forward(.75,1000);
            Forward(-1,500);
            if (frontStage) {
                Strafe(1,500, !blue);
                sleep(100);
                Forward(.5,1250);
                sleep(100);
                Strafe(.5, 3500, blue);
                Forward(-1,320);
                Strafe(1, 900, blue);
            }
            else{
                Strafe(1,1000,blue);
            }
        }
        // Is the object close to the truss?
        if ((frontStage && !blue && objLoc.equals("R")) ||
                (frontStage && blue && objLoc.equals("L")) ||
                (!frontStage && !blue && objLoc.equals("L")) ||
                (!frontStage && blue && objLoc.equals("R")))
        {
            Forward(.5, 875);
            Strafe(.25,390, !blue);
            tankTurn(.25,900 , !blue);
            Forward(.75,600 );
            Forward(-1,500);
            tankTurn(.25,600, blue);
            Forward(1,500);
            if (frontStage) {
                Strafe(1,500, !blue);
                sleep(100);
                Forward(.5,1000);
                sleep(100);
                Strafe(.5, 3500, blue);
                Forward(-1,300);
                Strafe(1, 900, blue);
            }
            else {
                Forward(-.25,1000);
                Strafe(.25, 2000, blue);
            }
        }

        // Fallback
        if (fallback) {
            if (objLoc.equals("L") || objLoc.equals("R")) {
                if (frontStage && !partial) {
                    Forward(1, 300);
                    Strafe(1, 500, !blue);
                    sleep(100);
                    Forward(.5, 2350);
                    sleep(100);
                    Strafe(.5, 3750, blue);
                    Forward(-.5, 400);
                    Strafe(.25, 4500, blue);
                }
                if (!frontStage && !partial) {
                    Forward(.4, 50);
                    Strafe(1, 1000, blue);
                }
                if (partial) {
                    Forward(.5, 2000);
                    sleep(100);
                    Forward(-1, 300);
                    sleep(250);
                }
            }
        }



        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }   // end runOpMode()

    //init the TFod
    private void initTFOD()     {
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                //Set Model
                .setModelFileName("ba.tflite")
                .setModelLabels(Labels)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.75f);
    }
}   // end class
 */