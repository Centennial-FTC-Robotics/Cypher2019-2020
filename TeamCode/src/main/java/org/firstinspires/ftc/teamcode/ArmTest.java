package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Arm Test", group="Test")
public class ArmTest extends CypherMethods {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        while(opModeIsActive()) {

            double HSlidePower = gamepad2.left_stick_y;
            double swivelPower = gamepad2.right_stick_x;
            boolean in = gamepad2.a;
            boolean out = gamepad2.b;
            boolean reset = gamepad2.y;

            HSlide.setPower(HSlidePower);
            swivel.setPower(swivelPower);

            if(in) {
                arm.setPower(1);
            }
            if(out) {
                arm.setPower(-1);
            }
            if(reset) {
                arm.setPower(0);
            }
        }





    }
}
