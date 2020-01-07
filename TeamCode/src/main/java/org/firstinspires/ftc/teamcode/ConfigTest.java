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

            for (DcMotor motor : driveMotors) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setPower(-.3);
                telemetry.addData("pos for " + motor.getPortNumber() ,motor.getCurrentPosition());
                telemetry.addData("weiughasdkgjhasdgsgd" + motor.getPortNumber(), motor.getPower());
            }
            telemetry.update();
        }

    }
}


