package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Config and Encoder Tester", group = "Test")
public class ConfigTest extends CypherMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        resetEncoders();

        waitForStart();
        while (opModeIsActive()) {
            for (DcMotor motor : driveMotors) {
                telemetry.addData("pos for " + motor.getPortNumber(), motor.getCurrentPosition());
            }
            telemetry.update();
        }

    }
}


