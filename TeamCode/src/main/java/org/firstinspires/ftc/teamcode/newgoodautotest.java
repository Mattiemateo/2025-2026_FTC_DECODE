package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous newgoodtest", group = "Autonomous")
@Configurable
public class newgoodautotest extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;

    private int pathState = 0;
    private Paths paths;

    // Updated to match the first point of the path
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));

    @Override
    public void init() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        pathState = 0;
    }

    @Override
    public void loop() {

        follower.update();
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));

        panelsTelemetry.update(telemetry);
    }

    public int autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                follower.setMaxPower(0.7);
                follower.followPath(paths.MainChain);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    pathState = 2;
                }
                break;
        }

        return pathState;
    }

    public static class Paths {

        public PathChain MainChain;

        public Paths(Follower follower) {

            MainChain = follower.pathBuilder()

                    .addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),
                                    new Pose(39.778, 35.036)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(180)
                    )

                    .addPath(
                            new BezierLine(
                                    new Pose(39.778, 35.036),
                                    new Pose(6.036, 35.343)
                            )
                    )
                    .setTangentHeadingInterpolation()

                    .addPath(
                            new BezierLine(
                                    new Pose(6.036, 35.343),
                                    new Pose(47.910, 34.897)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()

                    .addPath(
                            new BezierLine(
                                    new Pose(47.910, 34.897),
                                    new Pose(65.172, 83.890)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(140)
                    )

                    .addPath(
                            new BezierLine(
                                    new Pose(65.172, 83.890),
                                    new Pose(37.153, 31.784)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(270))

                    .build();
        }
    }
}