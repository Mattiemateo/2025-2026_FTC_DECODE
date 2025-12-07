package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Autonomous shoot", group = "EXPERIMENTAL!!!")
@Configurable // Panels
public class Autoshoot extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
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

        public PathChain leave1;
        public PathChain turn;
        public PathChain Intakeeeee;
        public PathChain turn2;
        public PathChain backtolaunch;
        public PathChain pewpewpew;
        public PathChain turn3;
        public PathChain Leave2park;

        public Paths(Follower follower) {
            leave1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 135.500), new Pose(48.000, 84.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            turn = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 84.000), new Pose(48.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180))
                    .build();

            Intakeeeee = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 84.000), new Pose(28.000, 84.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            turn2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(28.000, 84.000), new Pose(28.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(57))
                    .build();

            backtolaunch = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(28.000, 84.000), new Pose(48.000, 115.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            pewpewpew = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 115.000), new Pose(48.000, 115.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(57), Math.toRadians(140))
                    .build();

            turn3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 115.000), new Pose(48.000, 115.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(-90))
                    .build();

            Leave2park = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 115.000), new Pose(48.000, 75.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start - follow the path
                follower.followPath(paths.leave1);
                pathState = 1;
                break;

            case 1: // Wait for path to complete
                if (!follower.isBusy()) {
                    pathState = 2; // Move to next state (done)
                }
                break;

            case 2: // Done
                // Autonomous complete
                break;
        }

        return pathState;
    }
}