package org.firstinspires.ftc.teamcode.badApples.driver.extra;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static org.firstinspires.ftc.teamcode.badApples.core.FancyFormatting.*;

@Disabled
@TeleOp(name="Formatting Test", group="Debug")
public class formattingTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();

        String alpha1 = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
        telemetry.addData("Alphabet Bold", toBoldString(alpha1));
        telemetry.addData("Sentence", toBoldString("The ROBOT is NOT a wire"));
        telemetry.addData("Progress bar", toStringPercent(0.5));
        telemetry.addData("True", toStringBool(true));
        telemetry.addData("False", toStringBool(false));
        telemetry.addData("True (colored)", toStringBoolColored(true));
        telemetry.addData("False (colored)", toStringBoolColored(false));

        telemetry.update();
        sleep(5000);
    }
}
