package org.firstinspires.ftc.teamcode;

public class ReusableMethods {
    /*
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        resetEncoders();
        waitForStart();
        double P = 1d / 1333;        //Bill was here
        ElapsedTime controllerTimer = new ElapsedTime();
        final int miliTillReady = 250;
        int factor = 1;

        while (opModeIsActive()) {
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;
            boolean right = gamepad1.right_bumper;
            boolean left = gamepad1.left_bumper;
            boolean cont = gamepad1.start;


            if (controllerTimer.milliseconds() > miliTillReady) {
                if (a) {
                    P += 0.0005;
                    controllerTimer.reset();
                }
                if (y) {
                    P += 0.0001;
                    controllerTimer.reset();

                }
                if (right) {
                    P += 0.005;
                    controllerTimer.reset();

                }
                if (b) {
                    P -= 0.0005;
                    controllerTimer.reset();

                }
                if (x) {
                    P -= 0.0001;
                    controllerTimer.reset();

                }
                if (left) {
                    P -= 0.001;
                    controllerTimer.reset();

                }
                if (cont) {
                    findP(0, 22.75 * factor, P);
                    factor *= -1;
                    controllerTimer.reset();

                }
            }
            telemetry.addData("P", P);
            telemetry.update();
        }


    }

     */
}
