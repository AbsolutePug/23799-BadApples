
package org.firstinspires.ftc.teamcode.driver.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.AutoCore;
import org.firstinspires.ftc.teamcode.Core.RobotFunctionCore;

@Autonomous(name = "Autonomous Advanced", group = "Basic")
public class AutoAdv extends LinearOpMode {
    AutoCore autonomous = new AutoCore();
    private final ElapsedTime runtime = new ElapsedTime();
    enum StartingLocation {FAR, SHORT,UNDEFINED}
    enum TeamColor {RED, BLUE}

    @Override
    public void runOpMode(){
        autonomous.init(hardwareMap);
        autonomous.initGyro(hardwareMap);
        StartingLocation starting_location = StartingLocation.SHORT;
        TeamColor team_color = TeamColor.RED;


        telemetry.addData("Select starting location with either gamepad", "Short = D-pad down, Far = D-pad up");
        telemetry.addData("Select team color with either gamepad", "RED = B, BLUE = X");
        telemetry.addData("STARTING LOCATION", starting_location.name());
        telemetry.addData("TEAM COLOR", team_color.name());
        telemetry.update();
        while (!isStopRequested() && !isStarted()) {
            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                starting_location = StartingLocation.FAR;
                telemetry.addData("STARTING LOCATION", starting_location.name());
                break;
            }
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                starting_location = StartingLocation.SHORT;
                telemetry.addData("STARTING LOCATION", starting_location.name());
                break;
            }
        }
        telemetry.update();        while (!isStopRequested() && !isStarted()) {
            if (gamepad1.b || gamepad2.b) {
                team_color = TeamColor.RED;
                telemetry.addData("TEAM COLOR", team_color.name());
                break;
            }
            if (gamepad1.x || gamepad2.x) {
                team_color = TeamColor.BLUE;
                telemetry.addData("TEAM COLOR", team_color.name());
                break;
            }

        }
        telemetry.update();

        waitForStart();
        telemetry.clearAll();
        telemetry.addData("Runtime",runtime.milliseconds());
        autonomous.setBrakes(RobotFunctionCore.Brake.ENGAGED); // set the desired state for brakes

        switch (starting_location) {
            case SHORT:
                autonomous.setFlywheel(RobotFunctionCore.FlywheelSpeed.FAR);
                autonomous.setIntake(true);
                autonomous.move(0,0.25,1500);
                sleep(2500); // wait until target velocity is reached
                autonomous.shoot();

                autonomous.setFlywheel(RobotFunctionCore.FlywheelSpeed.OFF);
                autonomous.setIntake(false);
                autonomous.move(0,1,700);
                break;
            case FAR:
                autonomous.setFlywheel(1100);
                autonomous.setIntake(true);
                autonomous.move(0,-0.25,3000);
                sleep(2500);
                autonomous.shoot();
                autonomous.setIntake(false);
                autonomous.setFlywheel(RobotFunctionCore.FlywheelSpeed.OFF);
                switch (team_color) {
                    case BLUE:
                        autonomous.move(-0.4,0,5000);
                        break;
                    case RED:
                        autonomous.move(0.4,0,5000);
                        break;
                }
                break;
        }
    } // end runOpMode()
}