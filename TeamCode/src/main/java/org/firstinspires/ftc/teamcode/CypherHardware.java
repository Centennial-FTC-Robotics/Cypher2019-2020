package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public abstract class CypherHardware extends LinearOpMode {
    DcMotorEx leftUp;
    DcMotorEx leftDown;
    DcMotorEx rightUp;
    DcMotorEx rightDown;
    DcMotorEx vLeft;
    DcMotorEx vRight;
    DcMotorEx leftIntake;
    DcMotorEx rightIntake;
    CRServo HSlide;
    Servo lFoundation;
    Servo rFoundation;
    Servo arm;
    Orientation orientation = new Orientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES, 0, 0, 0, 0);
    BNO055IMU imu;
    double initialHeading;
    double initialPitch;
    double initialRoll;
    RevBlinkinLedDriver blinkinLed;

    List<LynxModule> hubs;


    @Override
    public void runOpMode() throws InterruptedException {
        getHardwareDevices();
        initHardware();
    }

    private void getHardwareDevices() {
        leftUp = hardwareMap.get(DcMotorEx.class, "upleft");
        rightUp = hardwareMap.get(DcMotorEx.class, "upright");
        rightDown = hardwareMap.get(DcMotorEx.class, "backright");
        leftDown = hardwareMap.get(DcMotorEx.class, "backleft");
        vLeft = hardwareMap.get(DcMotorEx.class, "vleft");
        vRight = hardwareMap.get(DcMotorEx.class, "vright");

        leftIntake = hardwareMap.get(DcMotorEx.class, "leftintake");
        rightIntake = hardwareMap.get(DcMotorEx.class, "rightintake");
        HSlide = hardwareMap.crservo.get("hslide");
        arm = hardwareMap.servo.get("armservo");
        lFoundation = hardwareMap.servo.get("foundation");
        rFoundation = hardwareMap.servo.get("foundation2");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        blinkinLed = hardwareMap.get(RevBlinkinLedDriver.class, "ledthingy");

        hubs = hardwareMap.getAll(LynxModule.class);


    }

    private void initHardware() {
        rightDown.setDirection(DcMotorEx.Direction.REVERSE);
        rightUp.setDirection(DcMotorEx.Direction.REVERSE);
        vLeft.setDirection(DcMotorEx.Direction.REVERSE);
        vRight.setDirection(DcMotorEx.Direction.REVERSE);
        lFoundation.setDirection(Servo.Direction.REVERSE);
        leftIntake.setDirection(DcMotorEx.Direction.REVERSE);

        leftDown.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftUp.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightDown.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightUp.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        vRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        vLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

    }

}
