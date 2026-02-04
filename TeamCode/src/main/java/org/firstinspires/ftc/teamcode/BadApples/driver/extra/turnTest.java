
package org.firstinspires.ftc.teamcode.BadApples.driver.extra;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BadApples.Core.AutoCore;
import org.firstinspires.ftc.teamcode.BadApples.Core.RobotFunctionCore;

@TeleOp(name = "Turn Test", group = "Basic")
public class turnTest extends LinearOpMode {
    AutoCore autonomous = new AutoCore();
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        autonomous.init(hardwareMap);
        autonomous.initGyro(hardwareMap);
        waitForStart();
        telemetry.addData("Runtime",runtime.milliseconds());
        autonomous.setBrakes(RobotFunctionCore.Brake.ENGAGED); // set the desired state for brakes



        autonomous.turn(90);
        sleep(2000);
        autonomous.turn(-90);
        sleep(1000);
    } // end runOpMode()
}