package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Robot;

@TeleOp(name = "TuffTeleTest")
public class TeleTest extends OpMode {
    Robot robot;
    private boolean rampUp = false;
    private boolean lastA = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.follower.setPose(OpModeStorage.autonomousEndPose);
        robot.follower.update();
    }

    @Override
    public void loop() {
        // Drive (field centric)
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                robot.follower.pose().heading()
        );
        robot.follower.manual(powers);

        // Relocalize
        if (gamepad1.startWasPressed()) {
            robot.follower.setPose(new Pose(9, 9, Math.toRadians(90)));
        }

        // Intake: set power directly, no Ivy needed
        if (gamepad1.left_bumper) {
            robot.intake.set(1, 1);      // run intake
        } else {
            robot.intake.set(0, 0);      // stop intake
        }
// Toggle ramp position with A
        if (gamepad1.a && !lastA) {
            rampUp = !rampUp;

            if (rampUp) {
//                robot.ramp.();    // Move servo to UP position
            } else {
                robot.rampdown();  // Move servo to DOWN position
            }//rampup is command builder so the commands dont doanything in teleop

        }

        lastA = gamepad1.a;
        robot.follower.update();

        Pose p = robot.follower.pose();
        telemetry.addData("X", p.x());
        telemetry.addData("Y", p.y());
        telemetry.addData("Heading", Math.toDegrees(p.heading()));
        telemetry.update();
    }
}