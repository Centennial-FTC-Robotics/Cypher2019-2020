package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class HelpU extends LinearOpMode {

    DcMotor leftUp;
    DcMotor leftDown;
    DcMotor rightUp;
    DcMotor rightDown;


    @Override
    public void runOpMode() throws InterruptedException {
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
            double leftPower = gamepad1.left_stick_x;
            double fowardPower = gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_x;
            if (rotate != 0) {
                double magnitude = Math.max(Math.abs(leftPower + fowardPower), Math.abs(leftPower - fowardPower));
                if (magnitude > 1) {
                    leftUp.setPower((-fowardPower + leftPower) / magnitude);
                    rightUp.setPower((fowardPower + leftPower) / magnitude);
                    leftDown.setPower((fowardPower + leftPower) / magnitude);
                    rightDown.setPower((-fowardPower + leftPower) / magnitude);
                } else {
                    leftUp.setPower(-fowardPower + leftPower);
                    rightUp.setPower(fowardPower + leftPower);
                    leftDown.setPower(fowardPower + leftPower);
                    rightDown.setPower(-fowardPower + leftPower);
                }
            } else {
                leftUp.setPower(rotate);
                rightDown.setPower(-rotate);
            }


        }
    }




}
