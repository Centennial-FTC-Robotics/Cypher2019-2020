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

        waitForStart();
        
        while(opModeIsActive()) {
            leftDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            leftDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //move 4000 encoder ticks no clue how far that is tho so ¯\_(ツ)_/¯
            leftUp.setTargetPosition(4000);
            leftDown.setTargetPosition(4000);
            rightUp.setTargetPosition(4000);
            rightDown.setTargetPosition(4000);

            leftUp.setPower(.5);
            leftDown.setPower(.5);
            rightDown.setPower(.5);
            rightUp.setPower(.5);

            while(areMotorsBusy() && opModeIsActive()) {

            }

            leftUp.setPower(0);
            leftDown.setPower(0);
            rightUp.setPower(0);
            rightDown.setPower(0);

        }


    }

    public boolean areMotorsBusy() {
        boolean busy;
        //easier way for this to work and look nicer but im lazy rn and wanna play minecraft so no
        if (leftDown.isBusy() || leftUp.isBusy() || rightUp.isBusy() || rightDown.isBusy()) {
            busy = true;
        } else {
            busy = false;
        }
        return busy;

    }

}
