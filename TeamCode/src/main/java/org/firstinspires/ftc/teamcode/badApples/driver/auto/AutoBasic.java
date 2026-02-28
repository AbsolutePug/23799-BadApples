
package org.firstinspires.ftc.teamcode.badApples.driver.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.badApples.core.AutoCore;
import org.firstinspires.ftc.teamcode.badApples.core.robotFunctionCore;

@Autonomous(name = "Autonomous: Move outside zone only", group = "Auto")
public class AutoBasic extends LinearOpMode {
    final AutoCore autonomous = new AutoCore();

    @Override
    public void runOpMode(){
        autonomous.init(hardwareMap);

        waitForStart();
        autonomous.setBrakes(robotFunctionCore.Brake.ENGAGED); // set the desired default state for brakes

        autonomous.move(0,1,1000);
        sleep(2000);
    } // end runOpMode()
}