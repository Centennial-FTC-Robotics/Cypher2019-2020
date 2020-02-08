package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public abstract class CypherHardware extends LinearOpMode {
    static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    static final String LABEL_FIRST_ELEMENT = "Stone";
    static final String LABEL_SECOND_ELEMENT = "Skystone";
    static final String VUFORIA_KEY = " AU4rZ23/////AAABmQabsAT5w0XtilSncDA5KR0mTpDy+NwTupFf3UHJK5uNazyphbkBUROQQ2ZmBNd5GDwgLEOA5XgeSxjo+pUUbNa85M03eRdF7I/O0083+YEIEORW45bjU4jNszzo5ASNn2Irz3QROUIg3T+1D8+H0n3AAt4ZL3f4P/zs+NsXPhaAhsE0lVn8EMEuXZm0jMoNhwp/cHISVhb0c4ZMywtCwMYR61l2oJLEvxIQmMC6AzKi2W8Ce+W8a2daBITha+t4FCLQgKCGTZG65/I24bdwW6aNt+Yd3HltnWnl13IKdZ5xJ0DDdM5i6x/8oMoqQfPxbOVnQez4dio31wAi7B23d42Ef2yJzTTRh1YFCRoy2aJY";
    protected TFObjectDetector tfod;
    DcMotorEx leftUp;
    DcMotorEx leftDown;
    DcMotorEx rightUp;
    DcMotorEx rightDown;
    DcMotorEx vLeft;
    DcMotorEx vRight;
    DcMotorEx leftIntake;
    DcMotorEx rightIntake;
    CRServo HSlide;
    CRServo swivel;
    Servo lFoundation;
    Servo rFoundation;
    Servo arm;
    Orientation orientation = new Orientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES, 0, 0, 0, 0);
    BNO055IMU imu;
    double initialHeading;
    double initialPitch;
    double initialRoll;
    VuforiaLocalizer vuforia;

    RevBlinkinLedDriver blinkinLed;



    @Override
    public void runOpMode() throws InterruptedException {
        getHardwareDevices();
        initHardware();
    }

    private void getHardwareDevices() {
        leftUp = hardwareMap.get(DcMotorEx.class,"upleft");
        rightUp = hardwareMap.get(DcMotorEx.class,"upright");
        rightDown = hardwareMap.get(DcMotorEx.class,"backright");
        leftDown = hardwareMap.get(DcMotorEx.class,"backleft");
        vLeft = hardwareMap.get(DcMotorEx.class,"vleft");
        vRight = hardwareMap.get(DcMotorEx.class,"vright");

        leftIntake = hardwareMap.get(DcMotorEx.class,"leftintake");
        rightIntake = hardwareMap.get(DcMotorEx.class,"rightintake");
        HSlide = hardwareMap.crservo.get("hslide");
        swivel = hardwareMap.crservo.get("swivelservo");
        arm = hardwareMap.servo.get("armservo");
        lFoundation = hardwareMap.servo.get("foundation");
        rFoundation = hardwareMap.servo.get("foundation2");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        blinkinLed = hardwareMap.get(RevBlinkinLedDriver.class, "ledthingy");

    }

    private void initHardware() {
        rightDown.setDirection(DcMotorEx.Direction.REVERSE);
        rightUp.setDirection(DcMotorEx.Direction.REVERSE);
        vLeft.setDirection(DcMotorEx.Direction.REVERSE);
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


    }

}
