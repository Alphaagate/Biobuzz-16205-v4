package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;


import com.pedropathing.math.Pose;

import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.pedro.mechanisms.Alliance;
import org.firstinspires.ftc.teamcode.pedro.mechanisms.Gate;
import org.firstinspires.ftc.teamcode.pedro.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.pedro.mechanisms.MovingShotCalculator;
import org.firstinspires.ftc.teamcode.pedro.mechanisms.Ramp;
import org.firstinspires.ftc.teamcode.pedro.mechanisms.ServoTurret;
import org.firstinspires.ftc.teamcode.pedro.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.pedro.mechanisms.Hood;
import java.security.AlgorithmParameterGenerator;

public class Robot {
    public final Intake intake;
    public final Shooter shooter;
    public final Ramp ramp;
    public final Gate gate;
    public final Hood hood;
    public final ServoTurret turret;
    public final MovingShotCalculator movingShot;
//    public final Gate gate;
    public final Follower follower;
    public Alliance alliance;

    private final Timer loop = new Timer();
    public double loops = 0, lastLoop = 0, loopTime = 0;
    public static Pose defaultPose = new Pose(8 + 24, 6.25 + 24, 0);
    public static Pose shootTarget = new Pose(ServoTurret.GOAL_X, ServoTurret.GOAL_Y, 0);

    public Robot(HardwareMap hardwareMap) {

//        l = new Limelight(h, a);
        shooter = new Shooter(hardwareMap);
        turret = new ServoTurret();
        turret.init(hardwareMap);   // ServoTurret uses init(), not a constructor
        ramp = new Ramp(hardwareMap);
        gate = new Gate(hardwareMap);
        hood = new Hood(hardwareMap);
        movingShot = new MovingShotCalculator(shooter, turret);
        follower = Constants.create(hardwareMap);
        intake = new Intake(hardwareMap);

        loop.reset();
//        setShootTarget();


    }


    public void saveEnd() {
        defaultPose = follower.poseAt(1);
    }


    public Pose getShootTarget() {
        return new Pose(ServoTurret.GOAL_X, ServoTurret.GOAL_Y, 0);
    }
    public CommandBuilder shoot() {
        return sequential(
                Commands.instant(shooter::on), // targets are updated continuously by MovingShotCalculator
                shooter.waitReady(),
                gateopen(),
                intake.intake(),              // feed balls
                waitMs(500),
                intake.off(),
                gateclose()
                );
    }

    public CommandBuilder shootOff() {
        return sequential(
                Commands.instant(shooter::off) // targets are updated continuously by MovingShotCalculator
        );
    }

    public CommandBuilder intake() {
        return sequential(
                intake.intake()
        );
    }

    public CommandBuilder rampup() {
        return sequential(
                ramp.up()
        );
    }

    public CommandBuilder rampdown() {
        return sequential(
                ramp.down()
        );
    }
    public CommandBuilder gateopen() {
        return sequential(
                gate.open()
        );
    }

    public CommandBuilder gateclose() {
        return sequential(
                gate.closed()
        );
    }

    public Command aimTurret(Follower follower) {
        return infinite(() -> turret.update(follower));
    }

    /** Updates the compensated turret target and flywheel target, then runs the shooter PIDF. */

    /**
     * Updates the moving-shot solution and runs the flywheel controller.
     * Call this once per OpMode loop, after the follower has updated.
     */
    public void updateMovingShot() {
        // Calculates the latest turret and flywheel targets.
        movingShot.update(follower);

        MovingShotCalculator.ShotSolution solution =
                movingShot.getSolution();

        // Do not keep commanding an old flywheel speed
        // when the current shot calculation is invalid.
        if (!solution.valid) {
            shooter.setTargetRpm(0.0);
        }

        // Uses the latest target and measured RPM to calculate motor power.
        shooter.periodic();
    }

    public CommandBuilder stopIntake() {
        return sequential(
//                gate.down(),
                intake.off()
        );
    }

    public double distanceToShootTarget() {

        Pose current = follower.pose();
        Pose target = getShootTarget();

        return Math.hypot(
                target.x() - current.x(),
                target.y() - current.y()
        );
    }

    public void updateHoodAim() {
        hood.hold();
    }
    public double getLoopTimeMs() {
        return loopTime;
    }

    public double getLoopTimeHz() {
        return 1000 / loopTime;
    }
}
