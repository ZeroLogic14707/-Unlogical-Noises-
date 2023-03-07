
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="Right Side: color Test3", group="Linear Opmode")
//defines everything 
public class Autonomaaaaus_test_test extends LinearOpMode{
    private DistanceSensor distanceF;
    private DistanceSensor distanceB;
    private DistanceSensor distanceR;
    private DistanceSensor distanceL;
    private ColorSensor color;
    private Blinker expansion_Hub_1;
    private Blinker expansion_Hub_2;
    private HardwareDevice webcam_1;
    private DcMotor armR;   // The Right Arm Motor (When looking at it from the back)
    //private DcMotor armL;  // The Left Arm Motor (When looking at it from the back)
    private Servo claw2;
    private Servo claw;
    //private DistanceSensor distance;
    private Gyroscope imu;
    private DcMotor LBD = null;
    private DcMotor LFD = null;
    private DcMotor RBD = null;
    private DcMotor RFD = null;
    //private DcMotor turret;
    /*private DcMotor         leftDrive   = null;
    private DcMotor         rightDrive  = null;*/

    private ElapsedTime     runtime = new ElapsedTime();
    
    // our actual gear ratio for drive motors is 18.8803:1 or 18.9:1
    // our actual gear ratio for the arm motor is 143.055667‬:1 or  143.1:1
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 9.2 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 1.515 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.7;
    static final double     STRAFE_SPEED            = 1;
    


    public final static double CLAW_HOME = 0.1;     // Starting position for claw 
    public final static double CLAW_MIN_RANGE = 0.15;  // Smallest number value allowed for claw position
    public final static double CLAW_MAX_RANGE = 0.8;  // Largestnumber value allowed for claw position
    
    double clawPosition = CLAW_HOME;  // Sets safe position
    final double CLAW_SPEED = 0.4 ;  // Sets rate to move servo
    
    public final static double CLAW2_HOME = 0.5;     // Starting position for claw 
    public final static double CLAW2_MIN_RANGE = 0.10;  // Smallest number value allowed for claw position
    public final static double CLAW2_MAX_RANGE = 0.7;  // Largestnumber value allowed for claw position
    
    double claw2Position = CLAW2_HOME;  // Sets safe position
    final double CLAW2_SPEED = 0.5 ;  // Sets rate to move servo
    
    // tell the drive motors to run to position
    public void RunToPosition() {
        RFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    // Reset the encoders for the drive motors
    public void ResetDriveEncoders() {
        LFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    // Reset the encoder for the arm
    public void ResetArmEncoder() {
        armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    // Reset the encoder for the arm
    /*public void ResetTurretEncoder() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }*/
    
    // Reset all the encoders on the robot
    public void ResetEncoders() {
        LFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    // Tell the robot to not use encoders
    public void StopUsingEncoders() {
        LFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //armL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    // Tell the robot to use encoders
    public void RunUsingEncoders() {
        LFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //armL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    // Send calculated power to the drive motors
    public void Power(double Power) {
        LFD.setPower(Power);
        LBD.setPower(Power);
        RFD.setPower(Power);
        RBD.setPower(Power);
    }
    
    // Send calculated power to the arm motor
    public void ArmPower(double Power) {
        armR.setPower(Power);
        //armL.setPower(Power);
    }

    // Tell the arm motor to run to calculated target position
    public void Arm(int Pos5) {
        armR.setTargetPosition(Pos5);
        //armL.setTargetPosition(Pos5);
    }
    
    public void encoderDrive(double speed, double LFInches, double RFInches, double LBInches, double RBInches, double timeoutS) {
        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLFTarget = LFD.getCurrentPosition() + (int)(LFInches * COUNTS_PER_INCH);
            newRFTarget = RFD.getCurrentPosition() + (int)(RFInches * COUNTS_PER_INCH);
            newLBTarget = LBD.getCurrentPosition() + (int)(LBInches * COUNTS_PER_INCH);
            newRBTarget = RBD.getCurrentPosition() + (int)(RBInches * COUNTS_PER_INCH);
            LFD.setTargetPosition(newLFTarget);
            RFD.setTargetPosition(newRFTarget);
            LBD.setTargetPosition(newLBTarget);
            RBD.setTargetPosition(newRBTarget);

            // Turn On RUN_TO_POSITION
            LFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            LFD.setPower(Math.abs(speed));
            RFD.setPower(Math.abs(speed));
            LBD.setPower(Math.abs(speed));
            RBD.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (LFD.isBusy() && RFD.isBusy() && LBD.isBusy() && RBD.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLFTarget,  newRFTarget);
                telemetry.addData("Running to",  " %7d :%7d", newLBTarget,  newRBTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                                            LFD.getCurrentPosition(), RFD.getCurrentPosition());
                telemetry.addData("Currently at",  " at %7d :%7d",
                                            LBD.getCurrentPosition(), RBD.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            LFD.setPower(0);
            RFD.setPower(0);
            LBD.setPower(0);
            RBD.setPower(0);

            // Turn off RUN_TO_POSITION
            LFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    
    // Send calculated power to the turret motor
    /*public void TurretPower(double Power) {
        turret.setPower(Power);
    }*/
    
    // Tell the turret motor to run to calculated target position
    /*public void Turret(int Pos5) {
        turret.setTargetPosition(Pos5);
    }*/
    
    // Tell the drive motors to run to calculated target position to strafe and/or control each motor independently
    public void MDrive(int Pos1, int Pos2, int Pos3, int Pos4) {
        LFD.setTargetPosition(Pos1);
        RFD.setTargetPosition(Pos2);
        LBD.setTargetPosition(Pos3);
        RBD.setTargetPosition(Pos4);
    }
    
    // Tell the drive motors to run to calculated target position to move forward and backwards and/or turn the robot
    public void Drive(int Pos1, int Pos2) {  // Pos1 controls the left side drive motors // Pos2 controls the right side drive motors
        LFD.setTargetPosition(Pos1);
        RFD.setTargetPosition(Pos2);
        LBD.setTargetPosition(Pos1);
        RBD.setTargetPosition(Pos2);
    }
    
    // Send the calculated velocity to the drive motors
    public void Velocity(double Power) {
        ((DcMotorEx) LFD).setVelocity(Power);
        ((DcMotorEx) LBD).setVelocity(Power);
        ((DcMotorEx) RFD).setVelocity(Power);
        ((DcMotorEx) RBD).setVelocity(Power);
    }
    
    // Send the calculated velocity to the arm motor
    public void armVelocity(double Power) {
        ((DcMotorEx) armR).setVelocity(Power);
        //((DcMotorEx) armL).setVelocity(Power);
    }
    
    // Send the calculated velocity to the turret motor
    /*public void turretVelocity(double Power) {
        ((DcMotorEx) turret).setVelocity(Power);
    }*/
    
    // Display on the driver hub that the arm motor is reaching for the "Low Junction"
    public void TelemetryL() {
        while (armR.isBusy()) {
            telemetry.addData("Stauts", "Low Junction");
            telemetry.update();
        }
    }
    
    // Display on the driver hub that the arm motor is reaching for the "Medium Junction"
    public void TelemetryM() {
        while (armR.isBusy()) {
            telemetry.addData("Stauts", "Medium Junction");
            telemetry.update();
        }
    }
    
    // Display on the driver hub that the arm motor is reaching for the "High Junction"
    public void TelemetryHi() {
        while (armR.isBusy()) {
            telemetry.addData("Stauts", "High Junction");
            telemetry.update();
        }
    }
    
    // Display on the driver hub that the arm motor is going back to it's "Home" position
    public void TelemetryH() {
        while (armR.isBusy()) {
            telemetry.addData("Stauts", "Home");
            telemetry.update();
        }
    }
    
    // Display on the driver hub what motors are active and what position all motors are at
    public void Telemetry() {
        while (opModeIsActive() && LFD.isBusy()) {   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
            telemetry.addData("encoder-fwd-left", LFD.getCurrentPosition() + "  busy=" + LFD.isBusy());
            telemetry.addData("encoder-fwd-right", RFD.getCurrentPosition() + "  busy=" + RFD.isBusy());
            telemetry.addData("encoder-bwd-left", LBD.getCurrentPosition() + "  busy=" + LBD.isBusy());
            telemetry.addData("encoder-bwd-right", RBD.getCurrentPosition() + "  busy=" + RBD.isBusy());
            telemetry.addData("encoder-arm", armR.getCurrentPosition() + "  busy=" + armR.isBusy());
            //telemetry.addData("encoder-arm", armL.getCurrentPosition() + "  busy=" + armL.isBusy());
            //telemetry.addData("encoder-arm", turret.getCurrentPosition() + "  busy=" + turret.isBusy());
            telemetry.update();
            //idle();
        }
    }
    
    // Display on the driver hub what motors are active and what position all motors are at  // Copy of the public void "Telemetry"
    public void Telemetry2() {
        while (opModeIsActive() && LFD.isBusy()) {   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
            telemetry.addData("encoder-fwd-left", LFD.getCurrentPosition() + "  busy=" + LFD.isBusy());
            telemetry.addData("encoder-fwd-right", RFD.getCurrentPosition() + "  busy=" + RFD.isBusy());
            telemetry.addData("encoder-bwd-left", LBD.getCurrentPosition() + "  busy=" + LBD.isBusy());
            telemetry.addData("encoder-bwd-right", RBD.getCurrentPosition() + "  busy=" + RBD.isBusy());
            telemetry.addData("encoder-arm-left", armR.getCurrentPosition() + "  busy=" + armR.isBusy());
            //telemetry.addData("encoder-arm-right", armL.getCurrentPosition() + "  busy=" + armL.isBusy());
            //telemetry.addData("encoder-turret", turret.getCurrentPosition() + "  busy=" + turret.isBusy());
            telemetry.update();
            //idle();
        }
    }
    
    
    // Tell the robot what to do if the robot senses a color // Use this public void to sense color
    public void Color() {
            while (opModeIsActive()) {
                //color sensor telemetry
                telemetry.addData("Red", color.red());
                telemetry.addData("Green", color.green());
                telemetry.addData("Blue", color.blue());
                //telemetry.addData("Yellow", color.toString());
                telemetry.update();
                
                // 
                if (color.red() > color.green() && color.red() > color.blue()) {
                    ResetDriveEncoders();
                    MDrive(0, 0, 0, 0);
                    RunToPosition();
                    Velocity(1);
                    Power(0.6);
                    Telemetry2();
                    
                    sleep(200);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    MDrive(-744, 744, 744, -744);/*  1116   1 inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.4); 
                    Telemetry2();
                    
                    sleep(1000);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(700);
                    
                    claw.setPosition(CLAW_MAX_RANGE);
                    claw2.setPosition(CLAW2_MIN_RANGE);
                    
                    sleep(1500);
                    
                    MDrive(-744, 744, 744, -744);/*  1116   1inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.4);
                    Telemetry2();
                    
                    sleep(1000000);
                    
                } else if (color.green() > color.red() && color.green() > color.blue()) {
                    ResetDriveEncoders();
                    MDrive(0, 0, 0, 0);
                    RunToPosition();
                    Velocity(1);
                    Power(0.6);
                    Telemetry2();
                    
                    sleep(200);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    MDrive(-744, 744, 744, -744);/*  1116   1 inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.4); 
                    Telemetry2();
                    
                    sleep(1000);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(700);
                    
                    claw.setPosition(CLAW_MAX_RANGE);
                    claw2.setPosition(CLAW2_MIN_RANGE);
                    
                    sleep(1500);
                    
                    MDrive(184, 184, 184, 184);/*  1116   1 inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.4); 
                    Telemetry2();
                    
                    sleep(1000);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(700);
                    
                    MDrive(2046, -2046, -2046, 2046);/*  1116   1inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.4);
                    Telemetry2();
                    
                   
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(1000000);
                    
                } else if (color.blue() > color.green() && color.blue() > color.red()) {
                    ResetDriveEncoders();
                    MDrive(0, 0, 0, 0);
                    RunToPosition();
                    Velocity(1);
                    Power(0.6);
                    Telemetry2();
                    
                    sleep(200);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(200);

                    MDrive(-744, 744, 744, -744);/*  1116   1 inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.4); 
                    Telemetry2();
                    
                    sleep(1000);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(700);
                    
                    claw.setPosition(CLAW_MAX_RANGE);
                    claw2.setPosition(CLAW2_MIN_RANGE);
                    
                    sleep(1500);
                    
                    MDrive(744, -744, -744, 744);/*  1116   1inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.4);
                    Telemetry2();
                    
                    sleep(1000000);
                }
            }
    }
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        distanceF = hardwareMap.get(DistanceSensor.class, "distance front");
        distanceB = hardwareMap.get(DistanceSensor.class, "distance back");
        distanceR = hardwareMap.get(DistanceSensor.class, "distance right");
        distanceL = hardwareMap.get(DistanceSensor.class, "distance left");
        color = hardwareMap.get(ColorSensor.class, "color");
        LFD = hardwareMap.get(DcMotor.class, "left front drive");
        LBD = hardwareMap.get(DcMotor.class, "left back drive");
        RFD = hardwareMap.get(DcMotor.class, "right front drive");
        RBD = hardwareMap.get(DcMotor.class, "right back drive");
        //turret = hardwareMap.get(DcMotor.class, "turret");
        armR = hardwareMap.get(DcMotor.class, "arm right");
        //armL = hardwareMap.get(DcMotor.class, "arm left");
        claw = hardwareMap.get(Servo.class, "claw");
        claw2 = hardwareMap.get(Servo.class, "claw2");

       
        LFD.setDirection(DcMotor.Direction.REVERSE);
        LBD.setDirection(DcMotor.Direction.REVERSE);
        RFD.setDirection(DcMotor.Direction.FORWARD);
        RBD.setDirection(DcMotor.Direction.FORWARD);
        armR.setDirection(DcMotor.Direction.FORWARD);
        //armL.setDirection(DcMotor.Direction.FORWARD);
        //turret.setDirection(DcMotor.Direction.FORWARD);
        LFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //armL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
       
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Start to run." );
        telemetry.update();
        
        ResetEncoders();
        
        waitForStart();
        
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  48,  48, 48, 48, 9.0);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   12, 12, -12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, -24, -24, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(100000);  // pause to display final telemetry message.
        
        //    LFD, RFD, LBD,  RBD
        /*MDrive(400, 400, 400, 400);/* 1333 1116   1 inch = 62 */
        /*RunToPosition();
        Velocity(1);
        Power(0.4);
        Telemetry2(); 
        
        sleep(600);
        
        LFD.setPower(0);
        RFD.setPower(0);
        LBD.setPower(0);
        RBD.setPower(0);
        ResetDriveEncoders();*/
    }
}