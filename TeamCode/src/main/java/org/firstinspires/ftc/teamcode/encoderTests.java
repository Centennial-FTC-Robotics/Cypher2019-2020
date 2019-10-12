package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class encoderTests extends LinearOpMode {
    
    DcMotor leftUp;
    DcMotor leftDown;
    DcMotor rightUp;
    DcMotor rightDown;

    @Override
    public void runOpMode() throws InterruptedException{
        leftUp =  hardwareMap.dcMotor.get("upleft");
        rightUp = hardwareMap.dcMotor.get("upright");
        rightDown = hardwareMap.dcMotor.get("backright");
        leftDown = hardwareMap.dcMotor.get("downleft");
        
        leftUp.setDirection(DcMotor.Direction.REVERSE);
        leftDown.setDirection(DcMotor.Direction.REVERSE);

        leftDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        
        while(opModeIsActive()) {

            leftDown.setTargetPosition(5000);


            
        }
    }


}
