package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Config Tester", group = "Test")
public class ConfigTest extends CypherMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        resetEncoders();
        waitForStart();

        while(opModeIsActive()) {
                leftDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("pos for " + leftDown.getPortNumber() ,leftDown.getCurrentPosition());
                telemetry.addData("weiughasdkgjhasdgsgd" + leftDown.getPortNumber(), leftDown.getPower());
            
            telemetry.update();
        }

    }
}


