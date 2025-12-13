
package org.firstinspires.ftc.teamcode.driver.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.AutoCore;
import org.firstinspires.ftc.teamcode.Core.RobotHardware;

@Autonomous(name = "Autonomous Basic", group = "Basic")
public class autonomous2026 extends LinearOpMode {
    AutoCore autonomous = new AutoCore();
    private final ElapsedTime runtime = new ElapsedTime();
    enum startingPosition{LEFT,RIGHT,UNDEFINED}

    @Override
    public void runOpMode(){
        autonomous.init(hardwareMap);
        startingPosition side = startingPosition.UNDEFINED;

        telemetry.addData("SIDE", side.name());
        telemetry.update();
        while (!isStopRequested() && !isStarted()) {
            if (gamepad1.dpad_left) {
                side = startingPosition.LEFT;
                telemetry.addData("SIDE", side.name());
                break;
            }
            if (gamepad1.dpad_right) {
                side = startingPosition.RIGHT;
                telemetry.addData("SIDE", side.name());
                break;
            }

        }
        telemetry.update();

        waitForStart();
        autonomous.setBrakes(RobotHardware.Brake.ENGAGED); // set the desired default state for brakes

        switch (side) {
            case LEFT:
                autonomous.move(0,1,1000);
                break;
            case RIGHT:
                autonomous.move(0,1,1000);
                break;
            default:
                autonomous.move(0,1,3000);
        }


        sleep(2000);
    } // end runOpMode()
}