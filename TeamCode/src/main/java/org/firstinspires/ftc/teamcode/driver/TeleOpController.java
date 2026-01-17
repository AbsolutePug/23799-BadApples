package org.firstinspires.ftc.teamcode.driver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.Core.FancyFormatting.*;

import org.firstinspires.ftc.teamcode.Core.RobotFunctionCore;

/**
 *  Written by Written by <a href="https://github.com/AbsolutePug">Robert Maddox (AbsolutePug)</a> 2025
 *  <p>
 *  ROBOT Controller
 */
@TeleOp(name="TeleOp Controller", group="TeleOp")
public class TeleOpController extends OpMode {
    RobotFunctionCore robot = new RobotFunctionCore();

    // Declare OpMode members for each of the motors
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime loop_time = new ElapsedTime();

    public void init() {
        // Initialize the robot
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Init, Ready for Start!");
        telemetry.update();

        robot.setBrakes(RobotFunctionCore.Brake.ENGAGED); // set the desired default state for brakes
        robot.setIntake(RobotFunctionCore.Intake.OFF);
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
        if (button_flywheel_far) robot.setFlywheel(RobotFunctionCore.FlywheelSpeed.FAR);
        else if (button_flywheel_short) robot.setFlywheel(RobotFunctionCore.FlywheelSpeed.SHORT);
        else if (button_flywheel_off) robot.setFlywheel(RobotFunctionCore.FlywheelSpeed.OFF);

        // Launcher
        robot.setLauncher(button_launcher_left,button_launcher_right);

        // Intake
        if (button_intake_on) robot.setIntake(RobotFunctionCore.Intake.ON);
        else if (button_intake_rebuke) robot.setIntake(RobotFunctionCore.Intake.REBUKE);
        else robot.setIntake(RobotFunctionCore.Intake.OFF);

        // Brake Control
        // If engaged the wheels will be locked in place, if not the wheels can be moved freely.
        if (button_brakes_on) robot.setBrakes(RobotFunctionCore.Brake.ENGAGED);
        else if (button_brakes_off) robot.setBrakes(RobotFunctionCore.Brake.DISENGAGED);

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
        robot.setPowers(
                leftFrontPower,
                rightFrontPower,
                leftBackPower,
                rightBackPower
        );

        // Basic Telemetry
        telemetry.addData           ("Status", runtime.toString());
        telemetry.addData           ("Loop time (ms)", loop_time.milliseconds());

        telemetry.addData           (toBoldString("CHASSIS"), "");
        telemetry.addData           ("Brakes", toStringBoolColored(robot.getBrakes()));
        telemetry.addData           ("Speed Multiplier", toStringPercent(numerator));

        telemetry.addData           (toBoldString("SCORING DEVICE"), "");
        telemetry.addData           ("Flywheel Velocity Raw", robot.getFlywheelVelocity());
        telemetry.addData           ("Flywheel Ready", toStringPercent(robot.getFlywheelReadyAsDecimal()));

        telemetry.addData           (toBoldString("EXTRA"), "");
        telemetry.addData           ("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData           ("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.update();
        loop_time.reset();
    }
}