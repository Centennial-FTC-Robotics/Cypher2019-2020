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
        super.runOpMode();
        zeroAngle();

        waitForStart();

        while(opModeIsActive()) {
           turnAbsolute(90);
            break;
        }
    }


}

