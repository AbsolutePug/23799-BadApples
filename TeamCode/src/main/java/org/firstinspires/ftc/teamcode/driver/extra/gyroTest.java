package org.firstinspires.ftc.teamcode.driver.extra;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.RobotHardware;

// This is a simple script to test the drive motors on the robot. This is used to tell if motor directions are set correctly.


@TeleOp(name="Gyro Tester", group="Test")
public class gyroTest extends OpMode {
    RobotHardware robot = new RobotHardware();

    public void init() {
        robot.initGyro(hardwareMap);
        robot.resetHeading();
    }

    public void loop() {
        telemetry.addData("Heading", robot.getHeading());
        telemetry.update();
    }
}
