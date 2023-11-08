package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DuckDrive {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo claw = null;
    private DcMotor pulley = null;
    private Servo wrist = null;

    HardwareMap hardwareMap;
    Telemetry telemetry;
    double COUNTS_PER_INCH;

    ElapsedTime runtime = null;

    //Constructor
    public DuckDrive(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        runtime = new ElapsedTime();
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");
        claw = hardwareMap.get(Servo.class, "claw");
        //shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        pulley = hardwareMap.get(DcMotor.class, "pulley");
        wrist = hardwareMap.get(Servo.class,"wrist");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        pulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
    // Note: pushing stick forward gives negative value
    public void driveMotor(double axial, double lateral, double yaw){
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    //axialInch: movement on the axial direction.
    //lateralInch: movement on the lateral direction.
    //yawDegree: clockwise yaw degree.
    public void driveByInch(MotorType motorType, double gear, double wheelDia,
                            double axialInch, double lateralInch, double yawDegree, double timeOut){

        // Calculate the COUNTS_PER_INCH for your specific drive train.
        // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
        // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
        // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
        // This is gearing DOWN for less speed and more torque.
        // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.

        double leftFrontInch  = axialInch + lateralInch + yawDegree;
        double rightFrontInch = axialInch - lateralInch - yawDegree;
        double leftBackInch   = axialInch - lateralInch + yawDegree;
        double rightBackInch  = axialInch + lateralInch - yawDegree;

        double COUNTS_PER_MOTOR_REV = 0 ;    // eg: TETRIX Motor Encoder
        double DRIVE_GEAR_REDUCTION = gear;     // No External Gearing.
        double WHEEL_DIAMETER_INCHES = wheelDia;     // For figuring circumference

        double DRIVE_SPEED = 0.6;
        double TURN_SPEED = 0.5;

        switch(motorType){
            case TETRIX:
                COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
                break;
            case NEVEREST40:
                COUNTS_PER_MOTOR_REV = 1120;    // eg: NeverRest40 Motor Encoder
                break;
            case NEVEREST20:
                COUNTS_PER_MOTOR_REV = 537.6;    // eg: NeverRest20 Motor Encoder
                break;
            case REV20:
                COUNTS_PER_MOTOR_REV = 560;    // eg: Rev20 Motor Encoder
                break;
            case REV40:
                COUNTS_PER_MOTOR_REV = 1120;    // eg: Rev40 Motor Encoder
                break;
            case GOBILDA312:
                COUNTS_PER_MOTOR_REV = 537.7;    // eg: goBilDa312 Motor Encoder
                break;
        }
        COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415927);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        runtime.reset();

        encoderDrive(0.2,leftFrontInch,rightFrontInch,leftBackInch,rightBackInch, timeOut);
    }

    public void setMotor(MotorType motorType, double gear, double wheelDia){

        // Calculate the COUNTS_PER_INCH for your specific drive train.
        // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
        // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
        // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
        // This is gearing DOWN for less speed and more torque.
        // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.


        double COUNTS_PER_MOTOR_REV = 0 ;    // eg: TETRIX Motor Encoder
        double DRIVE_GEAR_REDUCTION = gear;     // No External Gearing.
        double WHEEL_DIAMETER_INCHES = wheelDia;     // For figuring circumference

        //double DRIVE_SPEED = 0.6;
        //double TURN_SPEED = 0.5;
        switch(motorType){
            case TETRIX:
                COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
                break;
            case NEVEREST40:
                COUNTS_PER_MOTOR_REV = 1120;    // eg: NeverRest40 Motor Encoder
                break;
            case NEVEREST20:
                COUNTS_PER_MOTOR_REV = 537.6;    // eg: NeverRest20 Motor Encoder
                break;
            case REV20:
                COUNTS_PER_MOTOR_REV = 560;    // eg: Rev20 Motor Encoder
                break;
            case REV40:
                COUNTS_PER_MOTOR_REV = 1120;    // eg: Rev40 Motor Encoder
                break;
            case GOBILDA312:
                COUNTS_PER_MOTOR_REV = 537.7;    // eg: goBilDa312 Motor Encoder
                break;
        }
        COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415927);
    }

    public enum MotorType{
        TETRIX,
        NEVEREST40,
        NEVEREST20,
        REV40,
        REV20,
        GOBILDA312

    }

    public void encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches,
                             double leftBackInches, double rightBackInches,
                             //double axialInches, double lateralInches,
                             double timeoutS) {
        /*
        int newLeftTarget;
        int newRightTarget;
        */

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;


        // Determine new target position, and pass to motor controller
        int leftFrontMovements = (int)(leftFrontInches * COUNTS_PER_INCH);
        int rightFrontMovements = (int)(rightFrontInches * COUNTS_PER_INCH);
        int leftBackMovements = (int)(leftBackInches * COUNTS_PER_INCH);
        int rightBackMovements = (int)(rightBackInches * COUNTS_PER_INCH);

        newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + leftFrontMovements;
        newRightFrontTarget = rightFrontDrive.getCurrentPosition() + rightFrontMovements;
        newLeftBackTarget = leftBackDrive.getCurrentPosition() + leftBackMovements;
        newRightBackTarget = rightBackDrive.getCurrentPosition() + rightBackMovements;


        leftFrontDrive.setTargetPosition(newLeftFrontTarget);
        rightFrontDrive.setTargetPosition(newRightFrontTarget);
        leftBackDrive.setTargetPosition(newLeftBackTarget);
        rightBackDrive.setTargetPosition(newRightBackTarget);


        // Turn On RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        leftFrontDrive.setPower(Math.abs(speed));
        rightFrontDrive.setPower(Math.abs(speed));
        leftBackDrive.setPower(Math.abs(speed));
        rightBackDrive.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while ((runtime.seconds() < timeoutS) &&
                (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() &&
                        leftBackDrive.isBusy() && rightBackDrive.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Running to",  " %7d :%7d :%7d :%7d ",
                    newLeftFrontTarget,  newRightFrontTarget,
                    newLeftBackTarget,  newRightBackTarget);
            telemetry.addData("Currently at",  " at %7d :%7d",
                    leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(),
                    leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            telemetry.update();
        }

        StopPower();

    }

    public void encoderSingleDCMotor(double speed,
                                     DcMotor dcMotor, double inchTarget,
                                     double timeoutS) {


        int newTarget;

        // Determine new target position, and pass to motor controller
        int newMovements = (int)(inchTarget * COUNTS_PER_INCH);

        newTarget = dcMotor.getCurrentPosition() + newMovements;


        dcMotor.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // reset the timeout time and start motion.
        runtime.reset();
        dcMotor.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while ((runtime.seconds() < timeoutS) && dcMotor.isBusy() ) {

            // Display it for the driver.
            telemetry.addData("Running to",  " %7d  ",
                    newTarget);
            telemetry.addData("Currently at",  " %7d ",
                    dcMotor.getCurrentPosition());
            telemetry.update();
        }

        StopPower();

    }


    public void StopPower() {
        // Stop all motion;
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void gridMovement(String direction, double blockCount) {
        double speed = 0.3;
        double extrax=2/23.5;
        double extray=2/23.5;
        double moveInch=23.5*blockCount;
        setMotor(MotorType.GOBILDA312,1,3.7796 ); // Set drive wheel parameters.
        switch(direction) {
            case "v": //if direction of movement is Vertical (forward/backward, positive/negative)
                moveInch*=(1-extrax);
                encoderDrive(speed, moveInch, moveInch,
                        moveInch, moveInch, 10);
                break;
            case "h": //if direction of movement is Horizontal (right/left, positive/negative)
                moveInch*=(1+extray);
                encoderDrive(speed, moveInch, -moveInch,
                        -moveInch, moveInch, 10);
                break;
        }
    }
    public void gridRotate(double eighthTurns, double wheelX) {
        //1 eighthTurns = 45 degrees
        //wheelX is the distance between left front and right back
        double speed = 0.3;
        // 1.566 is a compensation factor at speed 0.3
        double moveInch = 3.1415927*wheelX/8*eighthTurns*1.56;
        setMotor (MotorType.GOBILDA312,1,3.7796 ); // Set drive wheel parameters.
        encoderDrive(speed, moveInch, -moveInch,
                moveInch, -moveInch, 5);
        //positive eighthTurns = clockwise
    }
    public void setGrip(boolean open) { //true = open claw, false = closed claw
        telemetry.addData("setGrip", open);
        telemetry.update();
        if(open){
            claw.setPosition(.35);
        }else{
            claw.setPosition(0.5);
        }
    }

    public void pulleyEncoderDrive(double speed,
                                   double inches,
                                   double timeoutS) {
        double     COUNTS_PER_MOTOR_REVpulley    = 1120; //Neverest 40
        double     DRIVE_GEAR_REDUCTIONpulley    = 1.0 ;     // No External Gearing.
        double     WHEEL_DIAMETER_INCHESpulley   = 1.3 ;     // For figuring circumference
        double     COUNTS_PER_INCHpulley         = (COUNTS_PER_MOTOR_REVpulley * DRIVE_GEAR_REDUCTIONpulley) /
                (WHEEL_DIAMETER_INCHESpulley * 3.1415);
        double     DRIVE_SPEEDpulley             = 0.3;
        int newTarget;

        // Determine new target position, and pass to motor controller
        newTarget = pulley.getCurrentPosition() + (int)(inches * COUNTS_PER_INCHpulley);
        pulley.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        pulley.setPower(Math.abs(speed));
        while ((runtime.seconds() < timeoutS) &&
                (pulley.isBusy() )) {
            pulley.getCurrentPosition();
            //telemetry.update();
        }

        // Stop all motion;
        //pulley.setPower(0);

        // Turn off RUN_TO_POSITION
        //pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void gridPulley(double inches) {

        double speed = 0.3; // Pulley speed
        //double moveInch = 3.1415927*wheelX/8*eighthTurns;
        setMotor (MotorType.TETRIX,1,1.3 ); // Set drive wheel parameters.
        encoderSingleDCMotor(speed, pulley, inches, 5);
        //pulleyEncoderDrive(0.3, inches, 0.5);
    }
    //above function^ max: 32 (increase later when we improve wiring), min: 0
    //high:31, ground:0

    public void setClaw(String status) {
        telemetry.addLine("Claw");
        telemetry.addData("setClaw", status);

        switch (status) {
            case ("Open"):
                claw.setPosition(0.35);
                telemetry.addData("value", 0.35);
                break;
            case ("Close"):
                claw.setPosition(0.55);
                telemetry.addData("value", 0.55);
                break;
        }
        telemetry.update();
    }

    public void setWrist(double position) {
        telemetry.addLine("Wrist:");
        telemetry.addData("setWrist", position);
        telemetry.update();
        wrist.setPosition(position);

    }

}
