package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Config Tester", group = "Test")
public class ConfigTest extends CypherMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();
        ElapsedTime help = new ElapsedTime();
        ElapsedTime nou = new ElapsedTime();
        double a = 0;
        double aa = 0;
        int count = 0;
        boolean hi = false;
        double max = 0;
        while(opModeIsActive()) {
           if(gamepad1.a) {
               help.reset();
               hi = true;
           }
           if(!gamepad1.a) {
               if(hi) {
                   a = help.nanoseconds();
                   aa += a;
                   count++;
                   hi = false;
                   help.reset();
               }
           }

           if(gamepad1.y) {
               count = 0;
           }

           if(a > max) {
               max = a;
           }

           telemetry.addData("let go of a", aa/count);
           telemetry.addData("max time", max);
           telemetry.update();
            }
        }
    }

