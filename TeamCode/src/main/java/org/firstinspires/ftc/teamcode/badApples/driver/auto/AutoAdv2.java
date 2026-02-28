
package org.firstinspires.ftc.teamcode.badApples.driver.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.badApples.core.AutoCore;
import org.firstinspires.ftc.teamcode.badApples.core.AutoCore.TeamColor;
import org.firstinspires.ftc.teamcode.badApples.core.AutoCore.StartingLocation;
import org.firstinspires.ftc.teamcode.badApples.core.robotFunctionCore;

@Autonomous(name = "Autonomous: Advanced", group = "Auto")
public class AutoAdv2 extends LinearOpMode {
    final AutoCore autonomous = new AutoCore();
    private final ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode(){
        autonomous.init(hardwareMap);
        autonomous.localization.initGyro(hardwareMap);
        StartingLocation starting_location = StartingLocation.SMALL_TRIANGLE;
        TeamColor team_color = TeamColor.BLUE;
        double team_color_coefficient = 1; // Invert movements

        telemetry.addData("Select starting location with either gamepad", "");
        telemetry.addData("Small Triangle = D-pad down", "Big Triangle = D-pad up");
        telemetry.addData("Select team color with either gamepad", "");
        telemetry.addData("RED = B","BLUE = X");
        telemetry.addData("STARTING LOCATION", starting_location.name());
        telemetry.addData("TEAM COLOR", team_color.name());
        telemetry.update();

        // Initializations options
        while (!isStopRequested() && !isStarted()) {
            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                starting_location = StartingLocation.BIG_TRIANGLE;
                telemetry.addData("STARTING LOCATION", starting_location.name());
                telemetry.update();
            }
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                starting_location = StartingLocation.SMALL_TRIANGLE;
                telemetry.addData("STARTING LOCATION", starting_location.name());
                telemetry.update();
            }
            if (gamepad1.b || gamepad2.b) {
                team_color = TeamColor.RED;
                telemetry.addData("TEAM COLOR", team_color.name());
                telemetry.update();
            }
            if (gamepad1.x || gamepad2.x) {
                team_color = TeamColor.BLUE;
                telemetry.addData("TEAM COLOR", team_color.name());
                telemetry.update();
            }
        }

        waitForStart();
        autonomous.resetTimer();
        if (team_color.equals(TeamColor.RED)) {
            team_color_coefficient = -1;
        } // swap relevant powers if on inverted side of field
        telemetry.clearAll();
        telemetry.addData("Runtime",runtime.milliseconds());
        autonomous.setBrakes(robotFunctionCore.Brake.ENGAGED); // set the desired state for brakes

        switch (starting_location) {
            case SMALL_TRIANGLE:
                // Shoot far
                autonomous.move(0,1,500);
                autonomous.turn(-40*team_color_coefficient);
                autonomous.setFlywheel(robotFunctionCore.FlywheelSpeed.FAR);
                autonomous.waitUntilFlywheelIsReady();
                sleep(1000);
                autonomous.setIntake(true);
                autonomous.shootLeft();
                autonomous.waitUntilFlywheelIsReady();
                sleep(1000);
                autonomous.shootRight();
                autonomous.setFlywheel(robotFunctionCore.FlywheelSpeed.OFF);
                autonomous.setIntake(false);
                autonomous.turn(40*team_color_coefficient);

                // Intake more
                autonomous.move(0,1,1000);
                autonomous.turn(-90*team_color_coefficient);
                autonomous.setIntake(true);
                autonomous.move(0,1,1250);
                autonomous.move(0,-0.5,2000);

                // Shoot short
                autonomous.move(0.75*team_color_coefficient,0,3000);
                autonomous.setFlywheel(robotFunctionCore.FlywheelSpeed.SHORT);
                autonomous.waitUntilFlywheelIsReady();
                sleep(1000);
                autonomous.setIntake(true);
                autonomous.shootLeft();
                autonomous.waitUntilFlywheelIsReady();
                autonomous.shootRight();
                autonomous.setFlywheel(robotFunctionCore.FlywheelSpeed.OFF);
                break;
            case BIG_TRIANGLE: //TODO: Unfinished
                autonomous.setFlywheel(robotFunctionCore.FlywheelSpeed.SHORT);
                autonomous.setIntake(true);
                autonomous.move(0,-0.25,3000);
                autonomous.waitUntilFlywheelIsReady();
                autonomous.shoot();
                autonomous.turn(45);
                autonomous.move(-1*team_color_coefficient,0, 1000);
                break;
        }
    } // end runOpMode()
}