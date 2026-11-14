package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Austin TeleOp")
public class TeleOp16205DriveCode extends OpMode {
    Robot robot;

    private Follower follower;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        follower = Constants.create(hardwareMap);
        robot.follower.setPose(OpModeStorage.autonomousEndPose);
        robot.follower.update();
    }

    @Override
    public void loop() {


        // Continuously aim turret
        robot.turret.update(robot.follower);

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                robot.follower.pose().heading()
        );
        ManualDrive.driveOrHold(robot.follower, powers);

        if (gamepad1.options) {
            Pose cornerPose = new Pose(10.5, 10.5, Math.toRadians(90));
            // On the fly Pose creation, we dont recommend this for Autonomous. Only accepts radians for heading
            robot.follower.setPose(cornerPose); // overrides our pose
        }

        robot.follower.update();
        Pose robotPose = robot.follower.pose(); // returns a Pose object
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        // Math.toDegrees() is a built-in java method
    }
}