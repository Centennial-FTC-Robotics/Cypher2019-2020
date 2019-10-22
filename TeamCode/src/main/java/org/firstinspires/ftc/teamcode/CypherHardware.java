package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class CypherHardware extends LinearOpMode {
    DcMotor leftUp;
    DcMotor leftDown;
    DcMotor rightUp;
    DcMotor rightDown;

    CRServo leftServo;
    CRServo rightServo;


    @Override
    public void runOpMode() throws InterruptedException  {
        leftUp =  hardwareMap.dcMotor.get("upleft");
        rightUp = hardwareMap.dcMotor.get("upright");
        rightDown = hardwareMap.dcMotor.get("backright");
        leftDown = hardwareMap.dcMotor.get("backleft");

        leftServo = hardwareMap.crservo.get("leftservo");
        rightServo = hardwareMap.crservo.get("rightservo");

        leftUp.setDirection(DcMotor.Direction.REVERSE);
        leftDown.setDirection(DcMotor.Direction.REVERSE);

        leftDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftServo.setDirection(CRServo.Direction.REVERSE);


    }

}
