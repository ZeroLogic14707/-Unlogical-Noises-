package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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

@Autonomous(name="Left Side: color circuit", group="Linear Opmode")

public class Leftside extends LinearOpMode{
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
    
    public void RunToPosition() {
        RFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void ResetDriveEncoders() {
        LFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void ResetArmEncoder() {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void ResetEncoders() {
        LFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void Power(double Power) {
        LFD.setPower(Power);
        LBD.setPower(Power);
        RFD.setPower(Power);
        RBD.setPower(Power);
    }
    public void ArmPower(double Power) {
        arm.setPower(Power);
    }
   
    public void Arm(int Pos5) {
        arm.setTargetPosition(Pos5);
        
    }
    public void ArmRuntoPsoition() {
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void StopUsingEncoders() {
        LFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void RunUsingEncoders() {
        LFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void MDrive(int Pos1, int Pos2, int Pos3, int Pos4) {
        LFD.setTargetPosition(Pos1);
        RFD.setTargetPosition(Pos2);
        LBD.setTargetPosition(Pos3);
        RBD.setTargetPosition(Pos4);
    }
    public void Drive(int Pos1, int Pos2) {
        LFD.setTargetPosition(Pos1);
        RFD.setTargetPosition(Pos2);
        LBD.setTargetPosition(Pos1);
        RBD.setTargetPosition(Pos2);
    }
    public void Velocity(double Power) {
        ((DcMotorEx) LFD).setVelocity(Power);
        ((DcMotorEx) LBD).setVelocity(Power);
        ((DcMotorEx) RFD).setVelocity(Power);
        ((DcMotorEx) RBD).setVelocity(Power);
        ((DcMotorEx) arm).setVelocity(Power);
    }
    public void TelemetryL() {
        while (arm.isBusy()) {
            telemetry.addData("Stauts", "Low Junction");
            telemetry.update();
        }
    }
    public void TelemetryM() {
        while (arm.isBusy()) {
            telemetry.addData("Stauts", "Medium Junction");
            telemetry.update();
        }
    }
    public void TelemetryHi() {
        while (arm.isBusy()) {
            telemetry.addData("Stauts", "High Junction");
            telemetry.update();
        }
    }
    public void TelemetryH() {
        while (arm.isBusy()) {
            telemetry.addData("Stauts", "Home");
            telemetry.update();
        }
    }
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
    public void Telemetry2() {
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
    public void Color() {
            while (opModeIsActive()) {
                //color sensor telemetry
                telemetry.addData("Red", color.red());
                telemetry.addData("Green", color.green());
                telemetry.addData("Blue", color.blue());
                telemetry.update();
                
                // 
                if (color.green() > color.red() && color.green() > color.blue()) {
                    ResetDriveEncoders();
                    MDrive(0, 0, 0, 0);
                    RunToPosition();
                    Velocity(1);
                    Power(0.3);
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
                    MDrive(-1100, -1100, -1100, -1100);
                    RunToPosition();
                    Velocity(1);
                    Power(0.3);
                    Telemetry2();
                    
                    sleep(400);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    MDrive(944, -944, 944, -944);/*  1116   1 inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.3); 
                    Telemetry2();
                    
                    sleep(700);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(700);
                    
                    ResetDriveEncoders();
                    MDrive(800, 800, 800, 800);
                    RunToPosition();
                    Velocity(1);
                    Power(0.3);
                    Telemetry2();
                    
                    sleep(400);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    claw.setPosition(CLAW_MIN_RANGE);
                    claw2.setPosition(CLAW2_MAX_RANGE);
                    
                    Arm(-500);  /* 90 = about 1 inch */
                    Velocity(0.5);
                    ArmPower(-0.6);
                    ArmRuntoPsoition();
                    TelemetryM();
                    
                    sleep(600);
                    
                    claw.setPosition(CLAW_MAX_RANGE);
                    claw2.setPosition(CLAW2_MIN_RANGE);
                    
                    
                    sleep(1500);
                    
                    Arm(2000);  /* 90 = about 1 inch */
                    Velocity(0.5);
                    ArmPower(0.6);
                    ArmRuntoPsoition();
                    TelemetryM();
                    
                    sleep(1300);
                    
                    MDrive(-2146, -2146, -2146, -2146);/*  1116   1inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.3);
                    Telemetry2();
                    
                    //sleep(3000);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(1500);
                    
                    MDrive(-1488, 1488, 1488, -1488);/*  1116   1 inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.3); 
                    Telemetry2();
                    
                    sleep(1000);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(700);
                    
                    sleep(1000000);
                } else if (color.red() > color.green() && color.red() > color.blue()) {
                    ResetDriveEncoders();
                    MDrive(0, 0, 0, 0);
                    RunToPosition();
                    Velocity(1);
                    Power(0.3);
                    Telemetry2();
                    
                    sleep(200);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    ResetDriveEncoders();
                    MDrive(-322, 322, -322, 322);
                    RunToPosition();
                    Velocity(1);
                    Power(0.3);
                    Telemetry2();
                    
                    sleep(400);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    MDrive(246, 246, 246, 246);/*  1116   1 inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.3); 
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
                    
                    
                    MDrive(1044, -1044, 1044, -1044);/*  1116   1 inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.3); 
                    Telemetry2();
                    
                    sleep(1000);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(700);
                    
                    MDrive(2046, 2046, 2046, 2046);/*  1116   1inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.3);
                    Telemetry2();
                    
                    //sleep(3000);
                    
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
                    Power(0.3);
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
                    Power(0.3);
                    Telemetry2();
                    
                    sleep(400);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    MDrive( -944, 944, -944, 944);/*  1116   1 inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.3); 
                    Telemetry2();
                    
                    sleep(700); 
                    
                     MDrive( 300, 300, 300, 300);/*  1116   1 inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.3); 
                    Telemetry2();
                    
                    sleep(300);
                    
                     Arm(-500);  /* 90 = about 1 inch */
                    Velocity(0.5);
                    ArmPower(-0.6);
                    ArmRuntoPsoition();
                    TelemetryM();
                    
                    sleep(500);
                    
                    claw.setPosition(CLAW_MAX_RANGE);
                    claw2.setPosition(CLAW2_MIN_RANGE);
                    
                    
                    sleep(1500);
                    
                    Arm(2000);  /* 90 = about 1 inch */
                    Velocity(0.5);
                    ArmPower(0.6);
                    ArmRuntoPsoition();
                    TelemetryM();
                    
                    sleep(1300);
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(700);
                    
                    ResetDriveEncoders();
                    MDrive(1000, 1000, 1000, 1000);
                    RunToPosition();
                    Velocity(1);
                    Power(0.3);
                    Telemetry2();
                    
                    sleep(400);
                    
                    
                    
                    LFD.setPower(0);
                    RFD.setPower(0);
                    LBD.setPower(0);
                    RBD.setPower(0);
                    ResetDriveEncoders();
                    
                    sleep(200);
                    
                    MDrive(0, 0, 0, 0);/*  1116   1 inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.3); 
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
                    
                    MDrive(1600, -1600, 1600, -1600);/*  1116   1inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.3);
                    Telemetry2();
                    
                    sleep(300);
                    
                    
                    MDrive(944, 944, 944, 944);/*  1116   1inch = 62 */
                    RunToPosition();
                    Velocity(1);
                    Power(0.3);
                    Telemetry2();
                    
                    sleep(300);
                    
                    sleep(1000000);
                }
            }
    }
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        color = hardwareMap.get(ColorSensor.class, "color");
        LFD  = hardwareMap.get(DcMotor.class, "left front drive");
        LBD  = hardwareMap.get(DcMotor.class, "left back drive");
        RFD = hardwareMap.get(DcMotor.class, "right front drive");
        RBD = hardwareMap.get(DcMotor.class, "right back drive");
        //turret = hardwareMap.get(DcMotor.class, "turret");
        arm = hardwareMap.get(DcMotor.class, "arm right");
        claw = hardwareMap.get(Servo.class, "claw");
        claw2 = hardwareMap.get(Servo.class, "claw2");

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
       
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        
        
        
        ResetEncoders();
        RunUsingEncoders();
        
        
        
        waitForStart();
        
        int Red; //= color.red();
        int Green; //= color.green();
        int Blue; //= color.blue();
        
        int color;
        
        
        
       
        claw.setPosition(CLAW_HOME);
        claw2.setPosition(CLAW2_HOME);
        
        sleep(2000);

        Arm(2145);  /* 90 = about 1 inch */
        Velocity(0.5);
        ArmPower(0.6);
        ArmRuntoPsoition();
        TelemetryM();
        
        sleep(500);
        
        
        claw.setPosition(CLAW_HOME);
        claw2.setPosition(CLAW2_HOME);
        
        sleep(600);
        
        LFD.setPower(0);
        RFD.setPower(0);
        LBD.setPower(0);
        RBD.setPower(0);
        ResetDriveEncoders();
        
        sleep(700);
        
        MDrive(1395, 1395, 1395, 1395);/* 1333 1116   1 inch = 62 */
        RunToPosition();
        Velocity(1);
        Power(0.4);
        Telemetry2(); 
        
        sleep(600);
        
        LFD.setPower(0);
        RFD.setPower(0);
        LBD.setPower(0);
        RBD.setPower(0);
        ResetDriveEncoders();
        
        Color();
        
        sleep(10000000);
        
         sleep(1000000);
       
    }
}