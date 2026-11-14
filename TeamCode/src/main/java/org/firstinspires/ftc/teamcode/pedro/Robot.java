package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
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
import org.firstinspires.ftc.teamcode.pedro.mechanisms.Ramp;
import org.firstinspires.ftc.teamcode.pedro.mechanisms.ServoTurret;
import org.firstinspires.ftc.teamcode.pedro.mechanisms.Shooter;

import java.security.AlgorithmParameterGenerator;

public class Robot {
    public final Intake intake;
    public final Shooter shooter;
    public final Ramp ramp;
    public final ServoTurret turret;
//    public final Gate gate;
    public final Follower follower;
    public Alliance alliance;

    private final Timer loop = new Timer();
    public double loops = 0, lastLoop = 0, loopTime = 0;
    public static Pose defaultPose = new Pose(8 + 24, 6.25 + 24, 0);
    public static Pose shootTarget = new Pose(6, 144 - 6, 0);

    public Robot(HardwareMap hardwareMap) {

//        l = new Limelight(h, a);
        shooter = new Shooter(hardwareMap);
        turret = new ServoTurret();
        turret.init(hardwareMap);   // ServoTurret uses init(), not a constructor

        ramp = new Ramp(hardwareMap);
        follower = Constants.create(hardwareMap);
        intake = new Intake(hardwareMap);


        loop.reset();
//        setShootTarget();


    }


    public void saveEnd() {
        defaultPose = follower.poseAt(1);
    }


//    public void setShootTarget() {
//        if (alliance == Alliance.BLUE && shootTarget.getX() != 6)
//            shootTarget = new Pose(6, 144 - 6, 0);
//        else if (alliance == Alliance.RED && shootTarget.getX() != (144 - 6))
//            shootTarget = shootTarget.mirror();
//    }

    public Pose getShootTarget() {
        return shootTarget;
    }

    public CommandBuilder shoot() {
        return sequential(
                shooter.waitReady(),          // wait until stable at target
                intake.intake(),              // feed balls
                Commands.waitMs(400.0),
                intake.off()
        );
    }

    public CommandBuilder spinUp() { return shooter.spinUp(); }
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


    public Command aimTurret(Follower follower) {
        return infinite(() -> turret.update(follower));
    }
    public CommandBuilder stopIntake() {
        return sequential(
//                gate.down(),
                intake.off()
        );
    }


    public double getLoopTimeMs() {
        return loopTime;
    }

    public double getLoopTimeHz() {
        return 1000 / loopTime;
    }
}