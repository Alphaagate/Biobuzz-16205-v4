
package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Robot;

@Configurable
public class OwenFlywheelTuner2 {
    private DcMotorEx outtake;
    Robot robot;

    public static double P = 0.005;
    public static double kV = 0.00041;
    public static double kS = 0.036;

    //1800 close 2020 far


    double velocity;

    public void init(HardwareMap hwMap) {
        outtake = hwMap.get(DcMotorEx.class, "outtakemotor");

        outtake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void shoot(double targetVelocity) {

        if (targetVelocity > 0) {

        velocity = (outtake.getVelocity());

        double error = targetVelocity - velocity;


        double feedback = error * P;
        double feedforward = kV * targetVelocity + kS;

        outtake.setPower(feedback + feedforward);}
        else {
            outtake.setPower(0);
        }


    }
    public double getVel(){return outtake.getVelocity();}
    public double getPoseX(){return robot.follower.pose().x();}
    public double getPoseY(){return robot.follower.pose().y();}





}