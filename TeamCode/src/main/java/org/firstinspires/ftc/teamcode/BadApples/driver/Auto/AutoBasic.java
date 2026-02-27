
package org.firstinspires.ftc.teamcode.BadApples.driver.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BadApples.Core.AutoCore;
import org.firstinspires.ftc.teamcode.BadApples.Core.RobotFunctionCore;

@Autonomous(name = "Autonomous: Move outside zone only", group = "Auto")
public class AutoBasic extends LinearOpMode {
    AutoCore autonomous = new AutoCore();
    private final ElapsedTime runtime = new ElapsedTime();
    enum startingPosition{FRONT,BACK,UNDEFINED}

    @Override
    public void runOpMode(){
        autonomous.init(hardwareMap);
        startingPosition side = startingPosition.UNDEFINED;

        telemetry.addData("SIDE", side.name());
        telemetry.update();
        while (!isStopRequested() && !isStarted()) {
            if (gamepad1.dpad_left) {
                side = startingPosition.FRONT;
                telemetry.addData("SIDE", side.name());
                break;
            }
            if (gamepad1.dpad_right) {
                side = startingPosition.BACK;
                telemetry.addData("SIDE", side.name());
                break;
            }

        }
        telemetry.update();

        waitForStart();
        autonomous.setBrakes(RobotFunctionCore.Brake.ENGAGED); // set the desired default state for brakes

        switch (side) {
            case FRONT:
                autonomous.move(0,1,1000);
                break;
            case BACK:
                autonomous.move(0,1,1000);
                break;
            default:
                autonomous.move(0,1,3000);
        }


        sleep(2000);
    } // end runOpMode()
}