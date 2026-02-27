package org.firstinspires.ftc.teamcode.badApples.driver.extra;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.badApples.core.LocalizationCore;
import org.firstinspires.ftc.teamcode.badApples.core.RobotFunctionCore;

@Autonomous(name="Gyro Tester", group="Test")
public class gyroTest extends OpMode {
    final RobotFunctionCore robot = new RobotFunctionCore();
    final LocalizationCore localization = new LocalizationCore();

    public void init() {
        localization.initGyro(hardwareMap);
        localization.resetHeading();
    }

    public void loop() {
        telemetry.addData("Heading", localization.getHeading());
        telemetry.update();
    }
}
