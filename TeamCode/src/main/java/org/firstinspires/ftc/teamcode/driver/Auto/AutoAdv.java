
package org.firstinspires.ftc.teamcode.driver.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.AutoCore;
import org.firstinspires.ftc.teamcode.Core.RobotHardware;

@Autonomous(name = "Autonomous Advanced", group = "Basic")
public class AutoAdv extends LinearOpMode {
    AutoCore autonomous = new AutoCore();
    private final ElapsedTime runtime = new ElapsedTime();
    enum Team {RED, BLUE,UNDEFINED}

    @Override
    public void runOpMode(){
        autonomous.init(hardwareMap);
        autonomous.initGyro(hardwareMap);
        Team side = Team.UNDEFINED;

        telemetry.addData("Select starting side with either gamepad, OR do not press either buttons to not turn the ROBOT", "RED = B, BLUE = X");
        telemetry.addData("SIDE", side.name());
        telemetry.update();
        while (!isStopRequested() && !isStarted()) {
            if (gamepad1.b || gamepad2.b) {
                side = Team.RED;
                telemetry.addData("SIDE", side.name());
                break;
            }
            if (gamepad1.a || gamepad2.a) {
                side = Team.BLUE;
                telemetry.addData("SIDE", side.name());
                break;
            }

        }
        telemetry.clearAll();

        waitForStart();
        telemetry.addData("Runtime",runtime.milliseconds());
        autonomous.setBrakes(RobotHardware.Brake.ENGAGED); // set the desired state for brakes

        switch (side) {
            case RED:
                autonomous.turn(45);
                break;
            case BLUE:
                autonomous.turn(-45);
                break;
            default:
                break;
        }
        autonomous.setFlywheel(RobotHardware.Flywheel.FAR);
        autonomous.setIntake(true);
        autonomous.move(0,0.25,1500);
        sleep(2500); // wait till reached target velocity
        autonomous.shoot();

        autonomous.setFlywheel(RobotHardware.Flywheel.OFF);
        autonomous.setIntake(false);
        autonomous.move(0,1,1000);


        sleep(2000);
    } // end runOpMode()
}