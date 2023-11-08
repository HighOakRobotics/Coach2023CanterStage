package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.DuckDrive;
@TeleOp(name="duck drive", group="Iterative OpMode")

public class TeleOpDuck extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    DuckDrive duckbot = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        duckbot = new DuckDrive(hardwareMap, telemetry);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        double offset = 0.0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double axial = -gamepad1.left_stick_y;
            double lateral= gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            duckbot.driveMotor(axial,lateral,yaw);

            if (gamepad1.dpad_up)
                duckbot.pulleyEncoderDrive(0.2, 2, 2);
            if (gamepad1.dpad_down)
                duckbot.pulleyEncoderDrive(0.2, -2, 2);

            if (gamepad1.b) duckbot.setClaw("Open");
            if (gamepad1.x) duckbot.setClaw("Close");
            if (gamepad1.y) duckbot.setWrist(0.7);
            if (gamepad1.a) duckbot.setWrist(0.8);
        }
    }
}
