package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
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

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

import static com.pedropathing.ivy.commands.Commands.lazy;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
@Autonomous

public class DriveTest extends LinearOpMode {

    Robot robot;
    protected Command updateShooter;
    private Command auto;
    private DcMotorEx outtake, outtake2, intake;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    //    private final PoseFactory p = PoseFactory.degrees().mirrorX(70.75);
//TODO: red and blue side switch, test which one is for red side and determine if you want to use control pad or make a separate opmode
    private final Pose pickupCornerStartPose = poseFactory.of(39, 9, 180);
    private final Pose pickupCornerPose = poseFactory.of(9, 9, 180);
    private final Pose shootFarPose = poseFactory.of(48.3591, 120, 90);
    private final Pose shootFarControl1Pose = poseFactory.of(26.2906, 101.0069, 0);
    private final Pose point3Pose = poseFactory.of(48.4023, 128.4746, 90);
    private final Pose intakeFlowerPose = poseFactory.of(12.9226, 46.9226, 180);
    private final Pose intakeFlowerControl1Pose = poseFactory.of(14, 98, 0);
    private final Pose intakeFlowerControl2Pose = poseFactory.of(50, 36, 0);
    private final Pose shootFlowerPose = poseFactory.of(48, 8, 90);
    private final Pose pickupLimelightPose = poseFactory.of(59.8554, 51.0381, 90);
    private final Pose shootLimelightPose = poseFactory.of(48.3992, 8.9979, 90);
    private final Pose parkPose = poseFactory.of(9.0867, 101.5183, 90);





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
    protected Path start;
    protected Path shootPreloads;
    protected Path pickupCorner;
    protected Path shootFar;
    protected Path path3;
    protected Path intakeFlower;
    protected Path shootFlower;
    protected Path pickupLimelight;
    protected Path shootLimelight;
    protected Path park;




    //TODO: see if this works


    private void generatePaths() {

//        shootPreloads = line(startPose, pickupCornerStartPose)
//                .linear(startPose, pickupCornerStartPose);
        pickupCorner = line(pickupCornerStartPose, pickupCornerPose).linear(pickupCornerStartPose, pickupCornerPose);//.with(Constants.foresightConfig.maxPathSpeed.at(0.5));

        shootFar = curve(pickupCornerPose, shootFarControl1Pose, shootFarPose).linear(pickupCornerPose, shootFarPose);//.with(Constants.foresightConfig.maxPathSpeed.at(0.5));

        path3 = line(shootFarPose, point3Pose).constant(point3Pose);//.with(Constants.foresightConfig.maxPathSpeed.at(0.5));

        intakeFlower = curve(point3Pose, intakeFlowerControl1Pose, intakeFlowerControl2Pose, intakeFlowerPose).linear(point3Pose, intakeFlowerPose);//.with(Constants.foresightConfig.maxPathSpeed.at(0.5));

        shootFlower = line(intakeFlowerPose, shootFlowerPose).linear(intakeFlowerPose, shootFlowerPose);//.with(Constants.foresightConfig.maxPathSpeed.at(0.5));

        pickupLimelight = line(shootFlowerPose, pickupLimelightPose).constant(pickupLimelightPose);//.with(Constants.foresightConfig.maxPathSpeed.at(0.5));

        shootLimelight =line(pickupLimelightPose, shootLimelightPose).constant(shootLimelightPose);//.with(Constants.foresightConfig.maxPathSpeed.at(0.5));

        park = line(shootLimelightPose, parkPose).linear(shootLimelightPose, parkPose);//.with(Constants.foresightConfig.maxPathSpeed.at(0.5));


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

        robot.follower.setPose(pickupCornerStartPose);

//        limelight = hardwareMap.get(
//                Limelight3A.class,
//                "limelight"
//        );
//
//        limelight.setPollRateHz(100);
//
//        // Pollen pipeline
//        limelight.pipelineSwitch(0);
//
//        limelight.start();

        generatePaths();
        auto = sequential(
                // robot.shoot(),

//                follow(robot.follower, shootPreloads),

                parallel(
                        follow(robot.follower, pickupCorner),
                        //.with(Constants.foresightConfig.maxPathSpeed.at(0.5))
                        robot.intake()
                ),

                //rampdown
//                followWithActionAtDistance(
//                        shootFar,
//                        20,
//                        //rampdown
//
//                        robot.shoot()
//                ),


                follow(robot.follower, shootFar),
                //robot.shoot
                robot.stopIntake(),



                parallel(
                        follow(robot.follower, intakeFlower),
                        robot.intake()
                ),

                robot.stopIntake(),
                //rampup

                follow(robot.follower, shootFlower),
//                robot.shoot(),
                parallel(
                        follow(robot.follower, pickupLimelight),
                        robot.intake()
                ),

                robot.stopIntake(),

                follow(robot.follower, shootLimelight),
//                robot.shoot(),

                follow(robot.follower, park)
        );
        telemetry.addLine("Initialized - Ready!");
        telemetry.update();
        waitForStart();


        schedule(auto);
        while (opModeIsActive()) {

            robot.follower.update();

            Scheduler.execute();





//            LLResult result = limelight.getLatestResult();

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
    }







}