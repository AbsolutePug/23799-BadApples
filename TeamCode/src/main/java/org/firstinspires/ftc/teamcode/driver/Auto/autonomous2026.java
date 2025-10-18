
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

    @Override
    public void runOpMode(){
        autonomous.init(hardwareMap);

        String side = "NONE";
        telemetry.addData("SIDE", "NONE");
        telemetry.update();
        while (!isStopRequested() && !isStarted()) {
            if (gamepad1.dpad_left) {
                side = "LEFT";
                telemetry.addData("SIDE", "LEFT");
                break;
            }
            if (gamepad1.dpad_right) {
                side = "RIGHT";
                telemetry.addData("SIDE", "RIGHT");
                break;
            }

        }
        telemetry.update();

        waitForStart();
        autonomous.setBrakes(true);

        if (side == "RIGHT") {
            autonomous.move(0.5,0.5,1900);
        } else if (side == "LEFT") {
            autonomous.move(-0.5,0.5,1900);
        }

        sleep(1000);
    } // end runOpMode()
}