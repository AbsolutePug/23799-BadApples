package org.firstinspires.ftc.teamcode.driver.extra;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.RobotFunctionCore;

// This is a simple script to test the drive motors on the robot. This is used to tell if motor directions are set correctly.


@TeleOp(name="Gyro Tester", group="Test")
public class gyroTest extends OpMode {
    RobotFunctionCore robot = new RobotFunctionCore();

    public void init() {
        robot.initGyro(hardwareMap);
        robot.resetHeading();
    }

    public void loop() {
        telemetry.addData("Heading", robot.getHeading());
        telemetry.update();
    }
}
