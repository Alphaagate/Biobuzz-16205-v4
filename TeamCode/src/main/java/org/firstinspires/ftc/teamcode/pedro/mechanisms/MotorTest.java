package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import android.util.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp
public class MotorTest extends LinearOpMode {

    public static double target = 0;
    public static double p1 = 0, p2 = 0, i = 0, d1 = 0, d2 = 0;
    public static double kp = 0.005, ki = 0, kd = 0.0001;
    double lastp1 = 0;
    double lastp2 = 0;
    int turretRotation = 0;

    private DcMotorEx intake, frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, outtake1;

    private IMU imu;

    @Override
    public void runOpMode() {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "fL");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "fR");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "bL");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "bR");

        outtake1 = hardwareMap.get(DcMotorEx.class, "out1");
        //outtake2 = hardwareMap.get(DcMotorEx.class, "out2");
        //turret = hardwareMap.get(DcMotorEx.class, "outA");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        frontLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorEx.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        double botHeading, rotX, rotY, denominator, x, y, rx;
        waitForStart();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("Status", "Running");
            //telemetry.addData("turretpos", turret.getCurrentPosition());

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (currentGamepad1.options) {
                imu.resetYaw();
            }

            if (currentGamepad2.dpad_up) {
                target = (2798 * 0.57);

            }
            else if (currentGamepad2.dpad_right) {
                target = (2798 * -0.222);
                intake.setPower(-0.6);
            }
            else if (currentGamepad2.dpad_left) {
                target = (2798 * 0.486);

            }
            else if (currentGamepad2.dpad_down) {
                target = (2798 * 0.4);

            } else  if (currentGamepad1.right_bumper) {
                //intake.setPower(-1);
                target = (2798 * -0.2);
            }

            else {
                target = (0.0);
                intake.setPower(0);
            }
            //launcherpid(target);


            if (currentGamepad2.left_stick_button) {
                turretRotation = 100;
            } else if (currentGamepad2.right_stick_button) {
                turretRotation = -100;
            } else {
                turretRotation = 0;
            }

            //turret.setTargetPosition(turret.getCurrentPosition() + turretRotation);
            //turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            //turret.setPower(0.7);

            if (currentGamepad1.left_trigger_pressed) {
                intake.setPower(-1);
            }
            else {
                intake.setPower(0);
            }
            if (currentGamepad1.left_bumper) {
                intake.setPower(1);
            }
            else {
                intake.setPower(0);
            }
            if (currentGamepad1.right_trigger_pressed) {
                outtake1.setPower(-1);
            }
            else {
                outtake1.setPower(0);
            }


            y = -gamepad1.left_stick_y;     // forward/back
            x = gamepad1.left_stick_x;      // strafe
            rx = gamepad1.right_stick_x * 0.8;  // rotation
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            // Strafing correction
            rotX *= 1.11;

            moveRobot(rotY, rotX, rx);
            //launcherpid(target);

        }
    }
    // public void launcherpid(double target) {
    //     double p1 = target - outtake1.getVelocity();
    //     double p2 = target - outtake2.getVelocity();

    //     double d1 = p1 - lastp1;
    //     double d2 = p2 - lastp2;

    //     double motorPower = (kp * (p1 + p2) / 2) + (kd * (d1 + d2) / 2);
    //     telemetry.addData("p1", p1);
    //     telemetry.addData("p2", p2);
    //     telemetry.addData("d1", d1);
    //     telemetry.addData("d2", d2);
    //     telemetry.addData("kp", kp);

    //     telemetry.addData("kd", kd);
    //     telemetry.addData("motorpower", motorPower);
    //     telemetry.addData("vel1", outtake1.getVelocity());
    //     telemetry.addData("vel2", outtake2.getVelocity());

    //     telemetry.update();
    //     outtake1.setPower(motorPower);
    //     outtake2.setPower(motorPower);
    //     lastp1 = p1;
    //     lastp2 = p2;
    // }
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double frontLeftPower    =  x + y + yaw;
        double frontRightPower   =  x - y - yaw;
        double backLeftPower     =  x - y + yaw;
        double backRightPower    =  x + y - yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels.
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }
}
