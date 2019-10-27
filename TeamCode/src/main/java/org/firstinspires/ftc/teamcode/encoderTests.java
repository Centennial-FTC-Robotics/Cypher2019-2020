package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class encoderTests extends /*LinearOpMode*/ CypherMethods {
    /*
        DcMotor leftUp;
        DcMotor rightUp;
        DcMotor rightDown;
        DcMotor leftDown;

     */
    @Override
    public void runOpMode() throws InterruptedException {
/*
        leftUp =  hardwareMap.dcMotor.get("upleft");
        rightUp = hardwareMap.dcMotor.get("upright");
        rightDown = hardwareMap.dcMotor.get("backright");
        leftDown = hardwareMap.dcMotor.get("backleft");

        rightUp.setDirection(DcMotor.Direction.REVERSE);
        rightDown.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();


*/
        while(opModeIsActive()) {
            autoMove(60,0,.5);
            break;
        }

/*

    }
    public void autoMove(double forward, double left, double power) {
        int forwardMovement = convertInchToEncoder(forward);
        int leftMovement = convertInchToEncoder(left);

        leftUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftUp.setTargetPosition(forwardMovement + leftMovement);
        rightUp.setTargetPosition(forwardMovement - leftMovement);
        leftDown.setTargetPosition(forwardMovement - leftMovement);
        rightDown.setTargetPosition(forwardMovement + leftMovement);

        leftUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("motor target" , forwardMovement);
        telemetry.addData("motor current" ,leftUp.getCurrentPosition());
        telemetry.update();

        leftUp.setPower(power);
        rightUp.setPower(power);
        leftDown.setPower(power);
        rightDown.setPower(power);



        waitForMotors();

        leftUp.setPower(0);
        rightUp.setPower(0);
        leftDown.setPower(0);
        rightDown.setPower(0);

        leftUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    public int convertInchToEncoder(double inches) {
        /*double ticksPerRotation = 383.6;
        double wheelDiameter = 3.937;
        double ticksPerWheelRotation = ticksPerRotation; //MULTIPLY BY 2 FOR ACTUAL ROBOT hktdzffd
        double distanceInWheelRotation = wheelDiameter * Math.PI;
        double ticksPerInch = distanceInWheelRotation/ticksPerWheelRotation;

        double encoderValue = inches/ticksPerInch;
        int intEncoderValue = (int) encoderValue;
        return intEncoderValue;
        return  (int) inches*31;


        }
    public void waitForMotors() {
        while(areMotorsBusy() && opModeIsActive()) {

        }
    }

    public boolean areMotorsBusy() {
        return leftDown.isBusy() || leftUp.isBusy() || rightUp.isBusy() || rightDown.isBusy();
    }
    */
    }
}
