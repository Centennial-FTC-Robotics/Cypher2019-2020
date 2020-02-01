package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Config Tester", group = "Test")
public class ConfigTest extends CypherMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        resetEncoders();

        waitForStart();
        ElapsedTime time = new ElapsedTime();
        while (opModeIsActive()) {
            for (DcMotor motor : driveMotors) {
                telemetry.addData("pos", motor.getCurrentPosition());
            }
            telemetry.update();
        }

    }
}


