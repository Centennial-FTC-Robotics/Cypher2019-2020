package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="servoTest", group="Test")
public class servoTest extends LinearOpMode
{
    CRServo leftServo;
    CRServo rightServo;

    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();
        while(opModeIsActive()) {
            controlIntakeServos(1);
            Thread.sleep  (5000);
            controlIntakeServos(-1)
        }
    }
}
