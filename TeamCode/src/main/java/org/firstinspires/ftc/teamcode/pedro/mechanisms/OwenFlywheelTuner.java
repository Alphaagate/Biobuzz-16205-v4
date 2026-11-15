package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Robot;

@TeleOp(name = "Austin owen flywheeltuner TeleOp")
public class OwenFlywheelTuner extends OpMode {
    Robot robot;

    private Follower follower;

    private OwenFlywheel shooter = new OwenFlywheel();


    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        follower = Constants.create(hardwareMap);
        robot.follower.setPose(OpModeStorage.autonomousEndPose);
        robot.follower.update();
        shooter.init(hardwareMap);

    }

    @Override
    public void loop() {
        shooter.shoot(1500);

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                robot.follower.pose().heading()
        );
        ManualDrive.driveOrHold(robot.follower, powers);

        if (gamepad1.options) {
            Pose cornerPose = new Pose(9, 9, Math.toRadians(90));
            // On the fly Pose creation, we dont recommend this for Autonomous. Only accepts radians for heading
            robot.follower.setPose(cornerPose); // overrides our pose
        }


        robot.follower.update();
        robot.updateMovingShot();
        Pose robotPose = robot.follower.pose(); // returns a Pose object
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        telemetry.addData("Shot target", robot.shooter.getTarget());
        telemetry.addData("Shot speed", robot.movingShot.getSolution().compensatedLaunchSpeed);
        telemetry.addData("Hood angle", Math.toDegrees(robot.movingShot.getSolution().hoodAngle));
        telemetry.addData("Shot valid", robot.movingShot.getSolution().valid);
        // Math.toDegrees() is a built-in java method
    }
}
