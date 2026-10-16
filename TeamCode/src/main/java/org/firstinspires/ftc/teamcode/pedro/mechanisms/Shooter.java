package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


import static com.pedropathing.ivy.Scheduler.schedule;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.path;


public class Shooter {
    private DcMotorEx left, right;

    private double target = 0;
    public static double kS = 0.08, kV = 0.00039, kP = 0.01;

    private boolean activated = true;

    public static final double NEAR = 1200;
    public static final double FAR = 1400;

    private static final double[] XS = {44, 72, 100};
    private static final double[] YS = {10, 38, 66};
    private static final double[][] CLOSE_VELOCITIES = {
            {1200, 1200, 1275},
            {1275, 1275, 1350},
            {1325, 1360, 1400}
    };
    public Shooter(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotorEx.class, "shooterleft");
        right = hardwareMap.get(DcMotorEx.class, "shooterright");
        left.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double getTarget() {
        return target;
    }

    public double getVelocity() {
        return right.getVelocity();
    }

    public void setPower(double p) {
        left.setPower(-p);
        right.setPower(-p);
    }

    public void off() {
        activated = false;
        setPower(0);
    }

    public void on() {
        activated = true;
    }

    public void shooterToggle() {
        activated = !activated;
        if (!activated)
            setPower(0);
    }

    public CommandBuilder toggle() {
        return Commands.instant(this::shooterToggle);
    }



    public void shootNear() {
        setTarget(NEAR);
        on();
    }

    public CommandBuilder shoot() {
        return Commands.instant(this::shootNear);
    }


    public void setTarget(double velocity) {
        target = velocity;
    }

    public void periodic() {
        if (activated)
            setPower((kV * getTarget()) + (kP * (getTarget() - getVelocity())) + kS);
    }

    public boolean atTarget() {
        return Math.abs((getTarget()- getVelocity())) < 50;
    }

    public void forDistance(double distance, boolean close) {
        //setTarget((6.13992 * distance) + 858.51272);
        if (close)
            //setTarget((0.000367066*Math.pow(distance,4))-(0.124995*Math.pow(distance,3))+(15.63311*Math.pow(distance,2))-(847.49369*distance)+18000.2274);
            setTarget((0.00180088*Math.pow(distance, 2))+(4.14265*distance)+948.97358);
        else
            setTarget(1550);
    }

    public String getLeftCurrent() {
        return "Left Shooter Motor: " + left.getCurrent(CurrentUnit.AMPS);
    }

    public String getRightCurrent() {
        return "Right Shooter Motor: " + right.getCurrent(CurrentUnit.AMPS);
    }

}