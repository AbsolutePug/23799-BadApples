package org.firstinspires.ftc.teamcode.driver;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *  Written by Written by <a href="https://github.com/AbsolutePug">Robert Maddox (AbsolutePug)</a> 2025
 *  <p>
 *  Version of TeleOp mode without usage of abstraction for illustration purposes. Do not use
 */
@Disabled
@TeleOp(name="TeleOp Controller Bad", group="TeleOp")
public class TeleOpControllerButBad extends OpMode {

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
    // Extra
    private boolean robot_active = true; // Whether the bot can be operated by controls, used for some automatic override

    // Gyro
    private IMU imu;

    public final double FLYWHEEL_TARGET_VELOCITY_FAR = 1500; // Target velocity for far goal
    public final double FLYWHEEL_TARGET_VELOCITY_SHORT = 1050; // Target velocity for far goal

    // Declare OpMode members for each of the motors
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime loop_time = new ElapsedTime();


    private static final double bar_width = 20; // The number of characters the progress bar is composed of // \u200B \u200C
    // Character unicodes
    private static final String unicode_green = "\uD83D\uDFE9";
    private static final String unicode_red = "\uD83D\uDFE5";
    private static final String unicode_a = "\uD83C\uDDE6";
    private static final String unicode_b = "\uD83C\uDDE7";
    private static final String unicode_c = "\uD83C\uDDE8";
    private static final String unicode_d = "\uD83C\uDDE9";
    private static final String unicode_e = "\uD83C\uDDEA";
    private static final String unicode_f = "\uD83C\uDDEB";
    private static final String unicode_g = "\uD83C\uDDEC";
    private static final String unicode_h = "\uD83C\uDDED";
    private static final String unicode_i = "\uD83C\uDDEE";
    private static final String unicode_j = "\uD83C\uDDEF";
    private static final String unicode_k = "\uD83C\uDDF0";
    private static final String unicode_l = "\uD83C\uDDF1";
    private static final String unicode_m = "\uD83C\uDDF2";
    private static final String unicode_n = "\uD83C\uDDF3";
    private static final String unicode_o = "\uD83C\uDDF4";
    private static final String unicode_p = "\uD83C\uDDF5";
    private static final String unicode_q = "\uD83C\uDDF6";
    private static final String unicode_r = "\uD83C\uDDF7";
    private static final String unicode_s = "\uD83C\uDDF8";
    private static final String unicode_t = "\uD83C\uDDF9";
    private static final String unicode_u = "\uD83C\uDDFA";
    private static final String unicode_v = "\uD83C\uDDFB";
    private static final String unicode_w = "\uD83C\uDDFC";
    private static final String unicode_x = "\uD83C\uDDFD";
    private static final String unicode_y = "\uD83C\uDDFE";
    private static final String unicode_z = "\uD83C\uDDFF";
    private static final String unicode_zwnj = "\u200C"; // used to prevent regional indicators from forming a flag (their entire purpose)


    /**
     *
     * @param value Decimal number between 0 and 1 to turn into a percentage value
     */
    public static String toStringPercent(double value) {
        double rounded_input = Math.min(Math.round(Math.abs(value)*bar_width), bar_width); // Turn the value from a decimal to an integer
        String result = "";

        for (int i = 0; i < rounded_input; i ++) { // Create the filled in bars
            result += "█";
        }
        for (int j = 0; j < bar_width-rounded_input; j++) { // Fill the rest with unfilled bars
            result += "░";
        }

        return(result);
    }
    public static String toStringBool(boolean value) {
        if (value) {
            return("██████████");
        } else {
            return("░░░░░░░░░░");
        }
    }
    public static String toStringBoolColored(boolean value) {
        if (value) {
            return(unicode_green+unicode_green+unicode_green+unicode_green+unicode_green+unicode_green+unicode_green+unicode_green+unicode_green+unicode_green+unicode_green+unicode_green+unicode_green+unicode_green+unicode_green);
        } else {
            return(unicode_red+unicode_red+unicode_red+unicode_red+unicode_red+unicode_red+unicode_red+unicode_red+unicode_red+unicode_red+unicode_red+unicode_red+unicode_red+unicode_red+unicode_red);
        }
    }

    /**
     * Make a string more bold using Unicode regional indicator symbols
     * @return a bold-er string
     */
    public static String toBoldString(String input) {
        String out = "";
        for (int i = 0; i < input.length(); i++) {
            out += toBoldChar(input.charAt(i)) + unicode_zwnj;
        }
        return out;
    }

    /**
     * @param in Character to make bolder
     * @return a unicode for a regional indicator with the same character
     */
    public static String toBoldChar(char in) {
        in = Character.toLowerCase(in);
        switch (in) {
            case 'a':
                return unicode_a;
            case 'b':
                return unicode_b;
            case 'c':
                return unicode_c;
            case 'd':
                return unicode_d;
            case 'e':
                return unicode_e;
            case 'f':
                return unicode_f;
            case 'g':
                return unicode_g;
            case 'h':
                return unicode_h;
            case 'i':
                return unicode_i;
            case 'j':
                return unicode_j;
            case 'k':
                return unicode_k;
            case 'l':
                return unicode_l;
            case 'm':
                return unicode_m;
            case 'n':
                return unicode_n;
            case 'o':
                return unicode_o;
            case 'p':
                return unicode_p;
            case 'q':
                return unicode_q;
            case 'r':
                return unicode_r;
            case 's':
                return unicode_s;
            case 't':
                return unicode_t;
            case 'u':
                return unicode_u;
            case 'v':
                return unicode_v;
            case 'w':
                return unicode_w;
            case 'x':
                return unicode_x;
            case 'y':
                return unicode_y;
            case 'z':
                return unicode_z;
            case ' ':
                return "   ";
            default:
                return "？";
        }
    }

    public void init() {
        // Initialize the robot
        // Assign motor variables
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        flywheel_1= hardwareMap.get(DcMotorEx.class, "launcher1");
        flywheel_2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        intake = hardwareMap.get(DcMotor.class, "intake");
        channel = hardwareMap.get(Servo.class, "blocker");
        launcher_right = hardwareMap.get(CRServo.class, "guider_1");
        launcher_left = hardwareMap.get(CRServo.class, "guider_2");

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

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Init, Ready for Start!");
        telemetry.update();

        leftFront   .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack    .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack   .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);
        runtime.reset();
    }

    public void loop() {
        // Controller Inputs
        double axial   = gamepad1.left_stick_y;     // Forward
        double lateral = gamepad1.left_stick_x;     // Strafe
        double yaw     = gamepad1.right_stick_x;    // Turn
        double trigger = gamepad1.right_trigger;    // Right Trigger (Slow-mode)
        boolean button_flywheel_off = gamepad2.left_bumper;
        boolean button_flywheel_short = gamepad2.b;
        boolean button_flywheel_far = gamepad2.y;
        boolean button_launcher_left = gamepad2.left_trigger>0.25;
        boolean button_launcher_right = gamepad2.right_trigger>0.25;
        boolean button_intake_on = gamepad2.a;
        boolean button_intake_rebuke = gamepad2.x;
        boolean button_brakes_on = gamepad1.a;
        boolean button_brakes_off = gamepad1.b;

        // Flywheel Control
        if (button_flywheel_far) {
            flywheel_1.setVelocity(1500);
            flywheel_2.setVelocity(-1500);
        }
        else if (button_flywheel_short) {
            flywheel_1.setVelocity(1050);
            flywheel_2.setVelocity(-1050);
        }
        else if (button_flywheel_off) {
            flywheel_1.setVelocity(1050);
            flywheel_2.setVelocity(-1050);
        }

        // Launcher
        if (button_launcher_left) {
            launcher_left.setPower(1);
        } else {
            launcher_left.setPower(0);
        }
        if (button_launcher_right) {
            launcher_right.setPower(1);
        } else {
            launcher_right.setPower(0);
        }

        // Intake
        if (button_intake_on) {
            intake.setPower(1);
        } else if (button_intake_rebuke) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        // Brake Control
        // If engaged the wheels will be locked in place, if not the wheels can be moved freely.
        if (button_brakes_on) {
            leftFront   .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack    .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack   .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else if (button_brakes_off) {
            leftFront   .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack    .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack   .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Speed multiplier
        double numerator = 1;
        if (trigger > 0.1) {
            numerator = 1 - (trigger / 2);
        }

        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1); // i stole this from a gm0 guide and i like it better because it fits on one line
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        double leftFrontPower  = (axial - lateral - yaw)*numerator/denominator;
        double rightFrontPower = (axial + lateral + yaw)*numerator/denominator;
        double leftBackPower   = (axial + lateral - yaw)*numerator/denominator;
        double rightBackPower  = (axial - lateral + yaw)*numerator/denominator;

        // Send calculated power to wheels
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

        // Basic Telemetry
        telemetry.addData           ("Status", runtime.toString());
        telemetry.addData           ("Loop time (ms)", loop_time.milliseconds());

        telemetry.addData           (toBoldString("CHASSIS"), "");
        telemetry.addData           ("Brakes", toStringBoolColored(false));
        telemetry.addData           ("Speed Multiplier", toStringPercent(numerator));

        telemetry.addData           (toBoldString("SCORING DEVICE"), "");
        telemetry.addData           ("Flywheel Velocity Raw", 0);
        telemetry.addData           ("Flywheel Ready", true);

        telemetry.addData           (toBoldString("EXTRA"), "");
        telemetry.addData           ("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData           ("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.update();
        loop_time.reset();
    }
}