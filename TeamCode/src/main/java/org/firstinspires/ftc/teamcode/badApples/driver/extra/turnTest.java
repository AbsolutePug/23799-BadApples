
package org.firstinspires.ftc.teamcode.badApples.driver.extra;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.badApples.core.AutoCore;
import org.firstinspires.ftc.teamcode.badApples.core.robotFunctionCore;

@Autonomous(name = "Turn Test", group = "Basic")
public class turnTest extends OpMode {
    final AutoCore autonomous = new AutoCore();
    private final ElapsedTime runtime = new ElapsedTime();

    public void init() {
        autonomous.init(hardwareMap);
        autonomous.localization.initGyro(hardwareMap);
    }

    public void loop() {
        telemetry.addData("Heading",autonomous.localization.getHeading());
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.addData("Runtime",runtime.milliseconds());
        autonomous.setBrakes(robotFunctionCore.Brake.ENGAGED); // set the desired state for brakes



        autonomous.turn(90);
        sleep(2000);
        autonomous.turn(-90);
        sleep(1000);
    } // end runOpMode()
}