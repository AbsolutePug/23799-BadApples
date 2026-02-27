package org.firstinspires.ftc.teamcode.BadApples.driver.extra;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BadApples.Core.LocalizationCore;
import org.firstinspires.ftc.teamcode.BadApples.Core.RobotFunctionCore;

@Autonomous(name="Gyro Tester", group="Test")
public class gyroTest extends OpMode {
    RobotFunctionCore robot = new RobotFunctionCore();
    LocalizationCore localization = new LocalizationCore();

    public void init() {
        localization.initGyro(hardwareMap);
        localization.resetHeading();
    }

    public void loop() {
        telemetry.addData("Heading", localization.getHeading());
        telemetry.update();
    }
}
