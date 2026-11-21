package org.firstinspires.ftc.teamcode.pedro;



import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.pedro.mechanisms.OwenFlywheelTuner2;
import org.firstinspires.ftc.teamcode.pedro.mechanisms.OwenIntake;

@Configurable
@TeleOp
public class testtunertele extends OpMode {

    // ============================================================================
    // MECHANISMS
    // ===========================================                                              `=================================

    // field
    private boolean shooterOn = false;


    private OwenFlywheelTuner2 shooter = new OwenFlywheelTuner2();
    private boolean shoot = false;
    public static double shootvel;
    private OwenIntake intake = new OwenIntake();


    // ============================================================================
    // SETTINGS
    // ============================================================================

    private static final double SLOW_MODE_FACTOR = 0.4;

    // ============================================================================
    // OPMODE LIFECYCLE
    // ============================================================================

    @Override
    public void init() {

        intake.init(hardwareMap);

        shooter.init(hardwareMap);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
//        if (gamepad1.aWasPressed()){
//            shooter.shoot(shootvel);
//        }

// in loop()
        if (gamepad1.aWasPressed()) {
            shooterOn = !shooterOn;
        }

        shooter.shoot(shooterOn ? shootvel : 0);

        // ====================================================================
        // DRIVING
        // ====================================================================



        // ====================================================================
        // INTAKE
        // ====================================================================
        if (gamepad1.rightBumperWasPressed()) {
                intake.allspin();

        } else if (gamepad1.dpadDownWasPressed()) {
            intake.intakeonly();
        } else if (gamepad1.rightBumperWasReleased() || gamepad1.dpadDownWasReleased()) {
            intake.stop();
        }



        telemetry.addData("target", shootvel);
        telemetry.addData("vel", shooter.getVel());
        telemetry.addData("error", shootvel - shooter.getVel());
        telemetry.update();


    }

    @Override
    public void stop() {}
}