package org.firstinspires.ftc.teamcode.pedro.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OwenIntake {
    private DcMotor intake, intake2;

    public void init(HardwareMap hwMap) {
        intake = hwMap.get(DcMotor.class, "intake");
        intake2 = hwMap.get(DcMotor.class, "transfer");


    }
    public void intakeonly(){
        intake.setPower(-1);
        intake2.setPower(0.2);
    }
    public void allspin(){
        intake.setPower(-1);
        intake2.setPower(1);
//        VectorLeft.setPower(-1);
//        VectorRight.setPower(1);
    }
    public void reverse(){
        intake.setPower(1);
        intake2.setPower(-1);
    }

    public void stop(){
        intake.setPower(0);
        intake2.setPower(0);
//        VectorLeft.setPower(0);
//        VectorRight.setPower(0);
    }
}
