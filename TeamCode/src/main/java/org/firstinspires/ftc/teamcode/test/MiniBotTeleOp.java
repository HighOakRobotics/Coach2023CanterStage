package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Minibot 13 teleop", group="Iterative OpMode")
// @Disabled
public class MiniBotTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    MiniBotHardware minibot = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        minibot = new MiniBotHardware(this, telemetry);
        minibot.init();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        double offset = 0.0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;

            if (gamepad1.dpad_down) offset = 0.0;
            if (gamepad1.dpad_left) offset = -0.2;
            if (gamepad1.dpad_right) offset = 0.2;

            minibot.driveRobot(drive, turn);

            minibot.setHandPositions(offset);

        }
    }
}
