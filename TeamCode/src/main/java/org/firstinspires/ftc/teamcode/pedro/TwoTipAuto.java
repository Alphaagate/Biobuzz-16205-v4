package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

//import com.pedropathing.robot.follower.Follower;
import com.pedropathing.paths.Path;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.math.Pose;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;

import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import static com.pedropathing.ivy.commands.Commands.waitUntil;

import org.firstinspires.ftc.teamcode.pedro.mechanisms.ServoTurret;

@Autonomous

public class TwoTipAuto extends LinearOpMode {

    Robot robot;
    protected Command updateShooter;
    private Command auto;
    private DcMotorEx outtake, outtake2, intake;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    //    private final PoseFactory p = PoseFactory.degrees().mirrorX(70.75);
//TODO: red and blue side switch, test which one is for red side and determine if you want to use control pad or make a separate opmode
    private final Pose startPose = poseFactory.of(39, 9, 90);
    private final Pose pickupCornerPose = poseFactory.of(9, 9, 180);
    private final Pose shootCornerStartPose = poseFactory.of(9, 9, 180);
    private final Pose shootCornerPose = poseFactory.of(39, 9, 180);
    private final Pose intakeFlowerPose = poseFactory.of(12, 48, 180);
    private final Pose intakeFlowerControl1Pose = poseFactory.of(40.1234, 52.3672, 0);
    private final Pose shootFarStartPose = poseFactory.of(12, 48, 180);
    private final Pose shootFarPose = poseFactory.of(48, 129, 90);
    private final Pose shootFarControl1Pose = poseFactory.of(31.5437, 51.0714, 0);
    private final Pose shootFarControl2Pose = poseFactory.of(6.6204, 111.3712, 0);
    private final Pose shootFarControl3Pose = poseFactory.of(50.3263, 95.8882, 0);
    private final Pose pickupLimelightPose = poseFactory.of(59.2357, 30.8386, -99.0469);
    private final Pose pickupLimelightControl1Pose = poseFactory.of(67.4289, 82.2959, 0);
    private final Pose pickupLimelightSegment1HeadingPose = poseFactory.of(59.2357, 30.8386, 90);
    private final Pose shootLimelightPose = poseFactory.of(48, 129.165, -70.3221);
    private final Pose shootLimelightControl1Pose = poseFactory.of(65.0249, 81.5582, 0);
    private final Pose parkPose = poseFactory.of(8.6721, 103.9427, 270);




    //    private Follower robot.follower;
    private Limelight3A limelight;
    // Limelight ball tracking settings
    private static final double TURN_KP = 0.035;
    private static final double MAX_TURN = 0.45;

    private static final double MAX_FORWARD = 0.35;
    private static final double TX_DEADBAND = 1.5;

    private static final double STOP_AREA = 1.0;
    private static final double AREA_KP = 0.40;

    // Stop trying after this many seconds.
    private static final double VISION_TIMEOUT_SECONDS = 3.5;

    private final ElapsedTime visionTimer = new ElapsedTime();

    private boolean visionFoundCloseBall = false;
    private double visionArea = 0.0;
    public double vel;

    protected Path pickupCorner;
    protected Path shootCorner;
    protected Path intakeFlower;
    protected Path shootFar;
    protected Path pickupLimelight;
    protected Path shootLimelight;
    protected Path park;


    private void generatePaths() {
//        shootPreloads = line(startPose, pickupCornerStartPose)
//                .linear(startPose, pickupCornerStartPose);

        pickupCorner = line(startPose, pickupCornerPose).constant(pickupCornerPose);

        shootCorner = line(shootCornerStartPose, shootCornerPose).linear(shootCornerStartPose, shootCornerPose);

        intakeFlower = Paths.curve(shootCornerPose, intakeFlowerControl1Pose, intakeFlowerPose).constant(intakeFlowerPose);

        shootFar = curve(shootFarStartPose, shootFarControl1Pose, shootFarControl2Pose, shootFarControl3Pose, shootFarPose).linear(shootFarStartPose, shootFarPose);

        pickupLimelight = curve(shootFarPose, pickupLimelightControl1Pose, pickupLimelightPose).heading(Interpolator.piecewise().until(0.114, Interpolator.constant(pickupLimelightSegment1HeadingPose)).until(1, Interpolator.tangent));

        shootLimelight = Paths.curve(pickupLimelightPose, shootLimelightControl1Pose, shootLimelightPose).constant(shootLimelightPose);//.with(Constants.foresightConfig.maxPathSpeed.at(0.5));

        park = line(shootLimelightPose, parkPose).constant(parkPose);//.with(Constants.foresightConfig.maxPathSpeed.at(0.5));
    }

    private Command followWithActionAtDistance(
            Path path,
            double distanceInches,
            Command action
    ) {
        // Make sure the path is long enough.
        if (distanceInches < 0 || distanceInches >= path.curve.length()) {
            throw new IllegalArgumentException(
                    "Trigger distance must be non-negative and less than path length."
            );
        }

        return parallel(
                // Follow the path normally.
                follow(robot.follower, path),

                // Wait until the specified distance has been traveled.
                sequential(
                        waitUntil(() ->
                                robot.follower.following()
                                        && robot.follower.currentCurve() != null
                                        && robot.follower.currentCurve().length()
                                        - robot.follower.remainingDistance()
                                        >= distanceInches
                        ),

                        // Execute the action at the distance marker.
                        action
                )
        );
    }

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        Scheduler.reset();
        robot.follower.setPose(startPose);

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );
        limelight.setPollRateHz(100);
        // Pollen pipeline
        limelight.pipelineSwitch(0);
        limelight.start();

        generatePaths();
        auto = sequential(
                robot.shoot(),
                waitMs(500.0),

                parallel(
                        follow(robot.follower, pickupCorner),
                        robot.intake()
                ),
                follow(robot.follower, shootCorner),
                robot.shoot(),
                waitMs(500.0),

                parallel(
                        robot.stopIntake(),
                        robot.rampdown()
                ),
                parallel(
                        follow(robot.follower, intakeFlower),
                        robot.intake()
                ),

                follow(robot.follower, shootFar),
                robot.shoot(),
                waitMs(500.0),

                parallel(
                        follow(robot.follower, pickupLimelight),
                        robot.intake()
                ),


                follow(robot.follower, shootLimelight),
                robot.shoot(),
                waitMs(500.0),

                follow(robot.follower, park)
        );
        telemetry.addLine("Initialized - Ready!");
        telemetry.update();
        waitForStart();

        schedule(auto);
        while (opModeIsActive()) {

            robot.follower.update();
            robot.shooter.periodic();                // REQUIRED every loop


            // Continuously aim turret
            robot.turret.update(robot.follower);

            // Run your Ivy scheduler
            Scheduler.execute();
            Scheduler.execute();

            if (robot.follower.pose().y() >= 70) {
                ServoTurret.GOAL_Y = 80.0;
            }
            else {
                ServoTurret.GOAL_Y = 60.0;
            }

//            LLResult result = limelight.getLatestResult();
            telemetry.addData("Shooter target", robot.shooter.getTarget());
            telemetry.addData("Shooter vel", robot.shooter.getVelocity());
            telemetry.addData("At target", robot.shooter.atTarget());
            telemetry.update();
            telemetry.addData(
                    "Follower Busy",
                    robot.follower.isBusy()
            );

            telemetry.addData(
                    "X",
                    robot.follower.pose().x()
            );

            telemetry.addData(
                    "Y",
                    robot.follower.pose().y()
            );

            telemetry.addData(
                    "Heading (deg)",
                    Math.toDegrees(robot.follower.pose().heading())
            );

            // Only added to let you verify that the Limelight is actually returning data.
//            if (result != null && result.isValid()) {
//                telemetry.addData(
//                        "Limelight",
//                        "Valid"
//                );
//
//                telemetry.addData(
//                        "Pollen blobs",
//                        result.getColorResults().size()
//                );
//            } else {
//                telemetry.addData(
//                        "Limelight",
//                        "No valid result"
//                );
//            }

            telemetry.update();
        }
        robot.shooter.off();
    }







}