package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Minibot: Auto Red 1 By Encoder", group="Robot")
// @Disabled
public class MiniBotAutoRed1 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    MiniBotHardware minibot = null;
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

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
        double distance = 24; //inches
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            minibot.encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            sleep(250);   // optional pause after each move.
            minibot.stop();
            sleep(1000);
            minibot.encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
            sleep(250);   // optional pause after each move.
            minibot.encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
            sleep(250);   // optional pause after each move.


        }
    }
}
