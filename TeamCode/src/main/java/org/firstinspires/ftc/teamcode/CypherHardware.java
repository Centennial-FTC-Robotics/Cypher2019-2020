package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public abstract class CypherHardware extends LinearOpMode {
    DcMotor leftUp;
    DcMotor leftDown;
    DcMotor rightUp;
    DcMotor rightDown;
    DcMotor vLeft;
    //Bill was here
    DcMotor vRight;

    CRServo leftServo;
    CRServo rightServo;
    CRServo HSlide;
    CRServo swivel;
    CRServo lFoundation, rFoundation;
    Servo arm;


    Orientation orientation = new Orientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES, 0, 0, 0, 0);
    BNO055IMU imu;

    double initialHeading;
    double initialPitch;
    double initialRoll;

    static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    static final String LABEL_FIRST_ELEMENT = "Stone";
    static final String LABEL_SECOND_ELEMENT = "Skystone";
    static final String VUFORIA_KEY =
            " AU4rZ23/////AAABmQabsAT5w0XtilSncDA5KR0mTpDy+NwTupFf3UHJK5uNazyphbkBUROQQ2ZmBNd5GDwgLEOA5XgeSxjo+pUUbNa85M03eRdF7I/O0083+YEIEORW45bjU4jNszzo5ASNn2Irz3QROUIg3T+1D8+H0n3AAt4ZL3f4P/zs+NsXPhaAhsE0lVn8EMEuXZm0jMoNhwp/cHISVhb0c4ZMywtCwMYR61l2oJLEvxIQmMC6AzKi2W8Ce+W8a2daBITha+t4FCLQgKCGTZG65/I24bdwW6aNt+Yd3HltnWnl13IKdZ5xJ0DDdM5i6x/8oMoqQfPxbOVnQez4dio31wAi7B23d42Ef2yJzTTRh1YFCRoy2aJY";

    VuforiaLocalizer vuforia;
    TFObjectDetector tfod;
    @Override
    public void runOpMode() throws InterruptedException {
        leftUp = hardwareMap.dcMotor.get("upleft");
        rightUp = hardwareMap.dcMotor.get("upright");
        rightDown = hardwareMap.dcMotor.get("backright");
        leftDown = hardwareMap.dcMotor.get("backleft");
        vLeft = hardwareMap.dcMotor.get("vleft");
        vRight = hardwareMap.dcMotor.get("vright");

        leftServo = hardwareMap.crservo.get("leftintake");
        rightServo = hardwareMap.crservo.get("rightintake");
        HSlide = hardwareMap.crservo.get("hslide");
        swivel = hardwareMap.crservo.get("swivelservo");
        arm = hardwareMap.servo.get("armservo");
        lFoundation = hardwareMap.crservo.get("foundation");
        rFoundation = hardwareMap.crservo.get("foundation2");


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        leftDown.setDirection(DcMotor.Direction.REVERSE);
        leftUp.setDirection(DcMotor.Direction.REVERSE);
        vLeft.setDirection(DcMotor.Direction.REVERSE);
        rightServo.setDirection(CRServo.Direction.REVERSE);

        leftDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}
