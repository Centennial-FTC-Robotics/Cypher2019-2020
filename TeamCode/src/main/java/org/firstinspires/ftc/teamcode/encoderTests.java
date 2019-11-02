package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@Autonomous
public class encoderTests extends CypherMethods{
    Orientation orientation = new Orientation(AxesReference.EXTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES,0,0,0,0);
    BNO055IMU imu;



    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        initialHeading = orientation.firstAngle;

        waitForStart();

        while(opModeIsActive()) {
           turnAbsolute(90);
            break;
        }
    }

    public int convertInchToEncoder(double inches) {
        /*double ticksPerRotation = 383.6;
        double wheelDiameter = 3.937;
        double ticksPerWheelRotation = ticksPerRotation; //MULTIPLY BY 2 FOR ACTUAL ROBOT hktdzffd
        double distanceInWheelRotation = wheelDiameter * Math.PI;
        double ticksPerInch = distanceInWheelRotation/ticksPerWheelRotation;

        double encoderValue = inches/ticksPerInch;
        int intEncoderValue = (int) encoderValue;
        return intEncoderValue; */
        return  (int) inches*31;


        }
    /*public void waitForMotors() {
        while(areMotorsBusy() && opModeIsActive()) {
            telemetry.addData("motor target" , for );
            telemetry.addData("motor current" ,leftUp.getCurrentPosition());
            telemetry.update();
        }
    }*/

    public boolean areMotorsBusy() {
        return leftDown.isBusy() || leftUp.isBusy() || rightUp.isBusy() || rightDown.isBusy();
    }

    public void autoMove(double forward, double right, double power) {
        int forwardMovement = convertInchToEncoder(forward);
        int rightMovement = convertInchToEncoder(right);

        leftUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftUp.setTargetPosition(forwardMovement + rightMovement);
        rightUp.setTargetPosition(forwardMovement - rightMovement);
        leftDown.setTargetPosition(forwardMovement - rightMovement);
        rightDown.setTargetPosition(forwardMovement + rightMovement);

        leftUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        leftUp.setPower(power);
        rightUp.setPower(power);
        leftDown.setPower(power);
        rightDown.setPower(power);

        while(areMotorsBusy() && opModeIsActive()) {
            telemetry.addData("forward target", forwardMovement );
            telemetry.addData("left target", rightMovement);
            telemetry.addData("motor current", leftUp.getCurrentPosition());
            telemetry.update();
        }
        leftUp.setPower(0);
        rightUp.setPower(0);
        leftDown.setPower(0);
        rightDown.setPower(0);

        leftUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


}

