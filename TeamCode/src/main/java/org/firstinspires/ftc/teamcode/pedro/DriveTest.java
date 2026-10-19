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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
@Autonomous

public class DriveTest extends LinearOpMode {

    Robot robot;
    protected Command updateShooter;
    private Command auto;
    private DcMotorEx outtake, outtake2, intake;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    //    private final PoseFactory p = PoseFactory.degrees().mirrorX(70.75);
//TODO: red and blue side switch, test which one is for red side and determine if you want to use control pad or make a separate opmode
    private final Pose startPose = poseFactory.of(48, 8, 180);
    private final Pose pickupCornerStartPose = poseFactory.of(48, 8, 180);
    private final Pose pickupCornerPose = poseFactory.of(8, 8, 180);
    private final Pose shootFarPose = poseFactory.of(47.6008, 129.3427, 90);
    private final Pose shootFarControl1Pose = poseFactory.of(37.3307, 103.2638, 0);
    private final Pose intakeFlowerPose = poseFactory.of(12, 48, 180);
    private final Pose shootFlowerPose = poseFactory.of(48, 8, 90);
    private final Pose pickupLimelightPose = poseFactory.of(59.8554, 51.0381, 74.5991);
    private final Pose shootLimelightPose = poseFactory.of(48.3992, 8.9979, 90);
    private final Pose parkPose = poseFactory.of(9.0867, 101.5183, 90);




//    private Follower robot.follower;
    private Limelight3A limelight;

    public double vel;
    protected Path start;
    protected Path shootPreloads;
    protected Path pickupCorner;
    protected Path shootFar;
    protected Path intakeFlower;
    protected Path shootFlower;
    protected Path pickupLimelight;
    protected Path shootLimelight;
    protected Path park;




    //TODO: see if this works


    private void generatePaths() {

//        shootPreloads = line(startPose, pickupCornerStartPose)
//                .linear(startPose, pickupCornerStartPose);
        pickupCorner = line(pickupCornerStartPose, pickupCornerPose)
                .linear(pickupCornerStartPose, pickupCornerPose)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.5));
        shootFar = curve(pickupCornerPose, shootFarControl1Pose, shootFarPose)
                .linear(pickupCornerPose, shootFarPose);
        intakeFlower = line(shootFarPose, intakeFlowerPose)
                .linear(shootFarPose, intakeFlowerPose);
        shootFlower = line(shootFarPose, shootFlowerPose)
                .linear(shootFarPose, shootFlowerPose);
        pickupLimelight = line(shootFlowerPose, pickupLimelightPose)
                .tangent();
        shootLimelight = line(pickupLimelightPose, shootLimelightPose)
                .constant(shootLimelightPose);
        park = line(shootLimelightPose, parkPose)
                .linear(shootLimelightPose, parkPose);

    }
//    Command autoStart = Command.build()
//            .setStart(() -> {
//                //turret
//                robot.shoot();
//            })
//            .setExecute(() -> {
//
//                robot.follower.follow(shootPreloads);
//
//
//            })
//            .setDone(() -> {
//                // return true to end the command
//                return true;
//            })
//            .setEnd(endCondition -> {
//                // executed on end
//            })
//            .requiring(/* requirements */);



    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        Scheduler.reset();

        robot.follower.setPose(startPose);

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
                        robot.intake()
                ),

                robot.stopIntake(),

                follow(robot.follower, shootFar),

                parallel(
                        follow(robot.follower, intakeFlower),
                        robot.intake()
                ),

                robot.stopIntake(),

                follow(robot.follower, shootFlower),

                parallel(
                        follow(robot.follower, pickupLimelight),
                        robot.intake()
                ),

                robot.stopIntake(),

                follow(robot.follower, shootLimelight),

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