package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class CypherHardware extends LinearOpMode {
    DcMotor leftUp;
    DcMotor leftDown;
    DcMotor rightUp;
    DcMotor rightDown;

    CRServo leftServo;
    CRServo rightServo;

    CRServo HSlide;
    CRServo swivel;
    CRServo arm;

    Orientation orientation = new Orientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES,0,0,0,0);
    BNO055IMU imu;

    double initialHeading;
    double initialPitch;
    double initialRoll;

    @Override
    public void runOpMode() throws InterruptedException  {
        leftUp =  hardwareMap.dcMotor.get("upleft");
        rightUp = hardwareMap.dcMotor.get("upright");
        rightDown = hardwareMap.dcMotor.get("backright");
        leftDown = hardwareMap.dcMotor.get("backleft");

        leftServo = hardwareMap.crservo.get("leftintake");
        rightServo = hardwareMap.crservo.get("rightintake");

        HSlide = hardwareMap.crservo.get("hslide");
        swivel = hardwareMap.crservo.get("swivelservo");
        arm = hardwareMap.crservo.get("armservo");


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        leftDown.setDirection(DcMotor.Direction.REVERSE);
        leftUp.setDirection(DcMotor.Direction.REVERSE);

        leftDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightServo.setDirection(CRServo.Direction.REVERSE);










    }

}
