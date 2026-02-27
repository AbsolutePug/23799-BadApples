package org.firstinspires.ftc.teamcode.pedroPathing.paths;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Test Path", group = "Autonomous")
@Disabled
@Configurable // Panels
public class TestPath extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain shootingFar;
        public PathChain Center;
        public PathChain shootingShort;

        public Paths(Follower follower) {
            shootingFar = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(60.000, 9.000),
                                    new Pose(60.000, 17.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))
                    .build();

            Center = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(60.000, 17.000),
                                    new Pose(72.000, 72.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(0))
                    .build();

            shootingShort = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(72.000, 72.000),
                                    new Pose(72.000, 135.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        switch (pathState) {
                case 0:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.shootingFar);
                        return 1;
                    }

                case 1:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Center);
                        return 2;
                    }
                case 2:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.shootingShort);
                        return -1;
                    }
        }

        follower.followPath(paths.Center);
        return pathState;
    }
}