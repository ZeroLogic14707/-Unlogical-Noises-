package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;

@Autonomous(name="Right Side: color", group="Linear Opmode")

public class Autonomaaaaus extends LinearOpMode{
    private ColorSensor color;
    private Blinker expansion_Hub_1;
    private Blinker expansion_Hub_2;
    private HardwareDevice webcam_1;
    private DcMotor arm;
    private Servo claw2;
    private Servo claw;
    private DistanceSensor distance;
    private Gyroscope imu;
    private DcMotor LBD;
    private DcMotor LFD;
    private DcMotor RBD;
    private DcMotor RFD;
    private DcMotor turret;

    // todo: write your code here
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
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    // Reset all the encoders on the robot
    public void ResetEncoders() {
        LFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    // Tell the robot to not use encoders
    public void StopUsingEncoders() {
        LFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    // Tell the robot to use encoders
    public void RunUsingEncoders() {
        LFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    // Send calculated power to the drive motors to strafe left
    public void SLPower(double Power) {
        LFD.setPower(1.2 * Power);
        LBD.setPower(1.1 * Power);
        RFD.setPower(1.2 * Power);
        RBD.setPower(1.2 * Power);
    }
    
    // Send calculated power to the drive motors to strafe right
    public void SRPower(double Power) {
        LFD.setPower(1.2 * Power);
        LBD.setPower(1.1 * Power);
        RFD.setPower(1.2 * Power);
        RBD.setPower(1.1 * Power);
    }
    
    // Send calculated power to the drive motors
    public void Power(double Power) {
        LFD.setPower(1.1 * Power);
        LBD.setPower(1.1 * Power);
        RFD.setPower(1.1 * Power);
        RBD.setPower(1.1 * Power);
    }
    
    public void zeroPower() {
        /*ResetDriveEncoders();
        LFD.setTargetPosition(0);
        MDrive(0, 0);
        Power(0.2);*/
        /*LFD.setZeroPowerBehavior(DcMotor.BRAKE);
        LBD.setZeroPowerBehavior(DcMotor.BRAKE);
        RFD.setZeroPowerBehavior(DcMotor.BRAKE);
        RBD.setZeroPowerBehavior(DcMotor.BRAKE);*/
        
        LFD.setPower(0);
        LBD.setPower(0);
        RFD.setPower(0);
        RBD.setPower(0);
    }
    
    public void Brake(double Power) {
        ResetDriveEncoders();
        //LFD.setTargetPosition(0);
        Drive(0, 0);
        Power(Power);
        Telemetry2();
        
    }
    
    // Send calculated power to the arm motor
    public void ArmPower(double Power) {
        arm.setPower(Power);
    }
    
    // Send calculated power to the turret
    /*public void Power(double Power) {
        turret.setPower(Power);
    }*/
    
    // Tell the arm motor to run to calculated target position
    public void Arm(int Pos5) {
        arm.setTargetPosition(Pos5);
    }
    
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
        ((DcMotorEx) arm).setVelocity(Power);
    }
    
    // Send the calculated velocity to the arm motor
    public void ArmVelocity(double Power) {
        ((DcMotorEx) arm).setVelocity(Power);
    }
    
    // Display on the driver hub that the arm motor is reaching for the "Low Junction"
    public void TelemetryL() {
        while (arm.isBusy()) {
            telemetry.addData("Stauts", "Low Junction");
            telemetry.update();
        }
    }
    
    // Display on the driver hub that the arm motor is reaching for the "Medium Junction"
    public void TelemetryM() {
        while (arm.isBusy()) {
            telemetry.addData("Stauts", "Medium Junction");
            telemetry.update();
        }
    }
    
    // Display on the driver hub that the arm motor is reaching for the "High Junction"
    public void TelemetryHi() {
        while (arm.isBusy()) {
            telemetry.addData("Stauts", "High Junction");
            telemetry.update();
        }
    }
    
    // Display on the driver hub that the arm motor is going back to it's "Home" position
    public void TelemetryH() {
        while (arm.isBusy()) {
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
            telemetry.addData("encoder-arm", arm.getCurrentPosition() + "  busy=" + arm.isBusy());
            telemetry.update();
            //idle();
        }
    }
    
    // Display on the driver hub what motors are active and what position all motors are at  // Copy of the public void "Telemetry"
    public void Telemetry2() {
        while (opModeIsActive() && LFD.isBusy() || RFD.isBusy() || LBD.isBusy() || RBD.isBusy() || arm.isBusy()) {   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
            telemetry.addData("encoder-fwd-left", LFD.getCurrentPosition() + "  busy=" + LFD.isBusy());
            telemetry.addData("encoder-fwd-right", RFD.getCurrentPosition() + "  busy=" + RFD.isBusy());
            telemetry.addData("encoder-bwd-left", LBD.getCurrentPosition() + "  busy=" + LBD.isBusy());
            telemetry.addData("encoder-bwd-right", RBD.getCurrentPosition() + "  busy=" + RBD.isBusy());
            telemetry.addData("encoder-arm", arm.getCurrentPosition() + "  busy=" + arm.isBusy());
            telemetry.update();
            //idle();
        }
    }
    
    // Tell the robot what to do if the robot senses a color
    /*public void ColorRed() {
            while (opModeIsActive()) {
                telemetry.addData("Red", color.red());
                telemetry.addData("Green", color.green());
                telemetry.addData("Blue", color.blue());
                
                //color sensor telemetry
                
                telemetry.update();
                
                if (color.red() > color.green() && color.red() > color.blue()) {
                    ResetDriveEncoders();
                    MDrive(300, 0, 0, 300);
                    RunToPosition();
                    Power(0.6);
                    Telemetry2();
                    sleep(1000);
                }
            }
            /*Green = color.green();
            Blue = color.blue();*/
    /*}
    /*public void ColorGreen() {
            while (opModeIsActive()) {
                telemetry.addData("Red", color.red());
                telemetry.addData("Green", color.green());
                telemetry.addData("Blue", color.blue());
                
                //color sensor telemetry
                
                telemetry.update();
                
                if (color.green() > color.red() && color.green() > color.blue()) {
                    ResetDriveEncoders();
                    MDrive(300, 300, 0, 0);
                    RunToPosition();
                    Power(0.6);
                    Telemetry2();
                    sleep(1000000);
                }
            }
            /*Green = color.green();
            Blue = color.blue();*/
    /*}
    public void ColorBlue() {
            while (opModeIsActive()) {
                telemetry.addData("Red", color.red());
                telemetry.addData("Green", color.green());
                telemetry.addData("Blue", color.blue());
                
                //color sensor telemetry
                
                telemetry.update();
                
                if (color.blue() > color.green() && color.blue() > color.red()) {
                    ResetDriveEncoders();
                    MDrive(0, 0, 300, 300);
                    RunToPosition();
                    Power(0.6);
                    Telemetry2();
                    sleep(1000);
                }
            }
            /*Green = color.green();
            Blue = color.blue();*/
    /*}*/
    
    // Tell the robot what to do if the robot senses a color // Use this public void to sense color
    public void Color() {
            while (opModeIsActive()) {
                //color sensor telemetry
                telemetry.addData("Red", color.red());
                telemetry.addData("Green", color.green());
                telemetry.addData("Blue", color.blue());
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
                    
                    Brake(0.2);
                    sleep(150);
                    zeroPower();
                    
                    /*LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);*/
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    /*MDrive(-744, 744, 744, -744);/*  1116   1 inch = 62 */
                    /*RunToPosition();
                    Velocity(1);
                    Power(0.4); 
                    Telemetry2();
                    
                    sleep(1000);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(700);*/
                    
                    ResetDriveEncoders();
                    MDrive(-1300, -1300, -1300, -1300);
                    RunToPosition();
                    Velocity(1);
                    Power(0.4);
                    Telemetry2();
                    
                    sleep(400);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    MDrive(-1488, 1488, 1488, -1488);/*  1116   1 inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.4); 
                    Telemetry2();
                    
                    sleep(700);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(700);
                    
                    ResetDriveEncoders();
                    MDrive(1300, 1300, 1300, 1300);
                    RunToPosition();
                    Velocity(1);
                    Power(0.4);
                    Telemetry2();
                    
                    sleep(400);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    MDrive(744, -744, -744, 744);/*  1116   1 inch = 62 */
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
                    
                    /*MDrive(-744, 744, 744, -744);/*  1116   1 inch = 62 */
                    /*RunToPosition();
                    Velocity(1);
                    Power(0.4); 
                    Telemetry2();
                    
                    sleep(1000);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(700);*/
                    
                    ResetDriveEncoders();
                    MDrive(-1300, -1300, -1300, -1300);
                    RunToPosition();
                    Velocity(1);
                    Power(0.4);
                    Telemetry2();
                    
                    sleep(400);
                    
                    Brake(0.2);
                    sleep(150);
                    zeroPower();
                    
                    /*LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);*/
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    MDrive(-1488, 1488, 1488, -1488);/*  1116   1 inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    SLPower(0.4); 
                    Telemetry2();
                    
                    sleep(700);
                    
                    Brake(0.2);
                    sleep(150);
                    zeroPower();
                    
                    /*LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);*/
                    ResetDriveEncoders();
                    
                    sleep(700);
                    
                    ResetDriveEncoders();
                    MDrive(1300, 1300, 1300, 1300);
                    RunToPosition();
                    Velocity(1);
                    Power(0.4);
                    Telemetry2();
                    
                    sleep(400);
                    
                    Brake(0.2);
                    sleep(150);
                    zeroPower();
                    
                    /*LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);*/
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    MDrive(744, -744, -744, 744);/*  1116   1 inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    SRPower(0.4); 
                    Telemetry2();
                    
                    sleep(1000);
                    
                    Brake(0.2);
                    sleep(150);
                    zeroPower();
                    
                    /*LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);*/
                    ResetDriveEncoders();
                    
                    sleep(700);
                    
                    claw.setPosition(CLAW_MAX_RANGE);
                    claw2.setPosition(CLAW2_MIN_RANGE);
                    
                    sleep(1500);
                    
                    /*MDrive(184, 184, 184, 184);/*  1116   1 inch = 62 */
                    /*RunToPosition();
                    Velocity(1);
                    Power(0.4); 
                    Telemetry2();
                    
                    sleep(1000);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(700);*/
                    
                    MDrive(2046, -2046, -2046, 2046);/*  1116   1inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    SRPower(0.4);
                    Telemetry2();
                    
                    //sleep(3000);
                    
                    Brake(0.2);
                    sleep(150);
                    zeroPower();
                    
                    /*LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);*/
                    ResetDriveEncoders();
                    
                    sleep(1000000);
                } else if (color.blue() > color.green() && color.blue() > color.red()) {
                    ResetDriveEncoders();
                    MDrive(0, 0, 0, 0);
                    RunToPosition();
                    Velocity(1);
                    Power(0.4);
                    Telemetry2();
                    
                    sleep(200);
                    
                    zeroPower();
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    /*MDrive(-744, 744, 744, -744);/*  1116   1 inch = 62 */
                    /*RunToPosition();
                    Velocity(1);
                    Power(0.4); 
                    Telemetry2();
                    
                    sleep(1000);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(700);*/
                    
                    ResetDriveEncoders();
                    MDrive(-1300, -1300, -1300, -1300);
                    RunToPosition();
                    Velocity(1);
                    Power(0.4);
                    Telemetry2();
                    
                    sleep(400);
                    
                    zeroPower();
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    MDrive(-1488, 1488, 1488, -1488);/*  1116   1 inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.4); 
                    Telemetry2();
                    
                    sleep(700);
                    
                    zeroPower();
                    ResetDriveEncoders();
                    
                    sleep(700);
                    
                    ResetDriveEncoders();
                    MDrive(1300, 1300, 1300, 1300);
                    RunToPosition();
                    Velocity(1);
                    Power(0.4);
                    Telemetry2();
                    
                    sleep(400);
                    
                    zeroPower();
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    MDrive(744, -744, -744, 744);/*  1116   1 inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.4); 
                    Telemetry2();
                    
                    sleep(1000);
                    
                    zeroPower();
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
        color = hardwareMap.get(ColorSensor.class, "color");
        LFD = hardwareMap.get(DcMotor.class, "left front drive");
        LBD = hardwareMap.get(DcMotor.class, "left back drive");
        RFD = hardwareMap.get(DcMotor.class, "right front drive");
        RBD = hardwareMap.get(DcMotor.class, "right back drive");
        //turret = hardwareMap.get(DcMotor.class, "turret");
        arm = hardwareMap.get(DcMotor.class, "arm right");
        claw = hardwareMap.get(Servo.class, "claw");
        claw2 = hardwareMap.get(Servo.class, "claw2");

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
        LFD.setDirection(DcMotor.Direction.REVERSE);
        LBD.setDirection(DcMotor.Direction.REVERSE);
        RFD.setDirection(DcMotor.Direction.FORWARD);
        RBD.setDirection(DcMotor.Direction.FORWARD);
        //turret.setDirection(DcMotor.Direction.FORWARD);
        //arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Wait for the game to start (driver presses PLAY)
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        
        ResetEncoders();
        
        waitForStart();
        
        int Red; //= color.red();
        int Green; //= color.green();
        int Blue; //= color.blue();
        int color;
        
        claw.setPosition(CLAW_HOME);
        claw2.setPosition(CLAW2_HOME);
        
        sleep(1700);
        
        Drive(0, 0);
        Arm(2145);  /* 90 = about 1 inch */
        Velocity(0.5);
        ArmPower(0.6);
        RunToPosition();
        TelemetryM();
        
        claw.setPosition(CLAW_HOME);
        claw2.setPosition(CLAW2_HOME);
        
        sleep(900);
        
        zeroPower();
        ResetDriveEncoders();
        
        sleep(700);
        
        MDrive(1395, 1395, 1395, 1395);/* 1333 1116   1 inch = 62 */
        RunToPosition();
        Velocity(1);
        Power(0.4);
        Telemetry2(); 
        
        sleep(600);
        
        Brake(0.2);
        sleep(150);
        zeroPower();
        
        /*LFD.setPower(0);
        RFD.setPower(0);
        LBD.setPower(0);
        RBD.setPower(0);*/
        ResetDriveEncoders();
        
        Color();
        
        sleep(10000000);
        
        /*if(Color(Red) > Color(Green) && Red > Blue){
             //sleep(700);
             
             ResetDriveEncoders();
             MDrive(300, 300, 300, 300);
             RunToPosition();
             Power(0.6);
             Telemetry2();
             sleep(1000);
             //movement 
             
             
         }/* else if(Green > Red && Green > Blue){
             ResetDriveEncoders();
             MDrive(192, 0, 192, 0);
             RunToPosition();
             Velocity(1);
             Power(0.6);
             Telemetry2();
             //sleep(1000);
         } else if(Blue > Green && Blue > Red){
             ResetDriveEncoders();
             MDrive(0, 0, 0, 192);
             RunToPosition();
             Velocity(1);
             Power(0.6);
             Telemetry2();
             //sleep(1000);
         }
        */
         sleep(1000000);
        // run until the end of the match (driver presses STOP)
    }
}