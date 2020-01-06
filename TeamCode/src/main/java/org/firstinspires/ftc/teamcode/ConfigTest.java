package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Config Tester", group = "Test")
public class ConfigTest extends CypherMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        while(opModeIsActive()) {

            for (DcMotor motor : driveMotors) {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setPower(.1);
                telemetry.addData("power", motor.getPower());
            }
            telemetry.update();
        }

    }
}


