
package org.firstinspires.ftc.teamcode.BadApples.driver.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BadApples.Core.AutoCore;
import org.firstinspires.ftc.teamcode.BadApples.Core.LocalizationCore;
import org.firstinspires.ftc.teamcode.BadApples.Core.RobotFunctionCore;
import org.firstinspires.ftc.teamcode.BadApples.Core.AutoCore.TeamColor;

@Autonomous(name = "Autonomous: Launch preloaded, pre-aimed", group = "Auto")
public class AutoAdv extends LinearOpMode {
    AutoCore autonomous = new AutoCore();
    LocalizationCore localization = new LocalizationCore();
    private final ElapsedTime runtime = new ElapsedTime();
    enum StartingLocation {BIG_TRIANGLE, SMALL_TRIANGLE,UNDEFINED}


    @Override
    public void runOpMode(){
        autonomous.init(hardwareMap);
        autonomous.localization.initGyro(hardwareMap);
        StartingLocation starting_location = StartingLocation.SMALL_TRIANGLE;
        TeamColor team_color = TeamColor.RED;
        double team_color_coefficient = 1; // Invert movements

        telemetry.addData("Select starting location with either gamepad", "Small Triangle = D-pad down, Big Triangle = D-pad up");
        telemetry.addData("Select team color with either gamepad", "RED = B, BLUE = X");
        telemetry.addData("STARTING LOCATION", starting_location.name());
        telemetry.addData("TEAM COLOR", team_color.name());
        telemetry.update();

        while (!isStopRequested() && !isStarted()) {
            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                starting_location = StartingLocation.BIG_TRIANGLE;
                telemetry.addData("STARTING LOCATION", starting_location.name());
                break;
            }
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                starting_location = StartingLocation.SMALL_TRIANGLE;
                telemetry.addData("STARTING LOCATION", starting_location.name());
                break;
            }
        }
        telemetry.update();
        while (!isStopRequested() && !isStarted()) {
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
        if (team_color.equals(TeamColor.RED)) {
            team_color_coefficient = -1;
        }
        telemetry.clearAll();
        telemetry.addData("Runtime",runtime.milliseconds());
        autonomous.setBrakes(RobotFunctionCore.Brake.ENGAGED); // set the desired state for brakes

        switch (starting_location) {
            case SMALL_TRIANGLE:
                autonomous.setFlywheel(RobotFunctionCore.FlywheelSpeed.FAR);
                autonomous.setIntake(true);
                autonomous.move(0,0.25,1500);
                autonomous.waitUntilFlywheelIsReady();
                autonomous.shoot();

                autonomous.setFlywheel(RobotFunctionCore.FlywheelSpeed.OFF);
                autonomous.setIntake(false);
                autonomous.move(0,1,700);
                break;
            case BIG_TRIANGLE: // TODO: make it good!
                autonomous.setFlywheel(RobotFunctionCore.FlywheelSpeed.SHORT);
                autonomous.setIntake(true);
                autonomous.move(0,-0.25,3000);
                autonomous.waitUntilFlywheelIsReady();
                autonomous.shoot();

                autonomous.turn(45);
                break;
        }
    } // end runOpMode()
}