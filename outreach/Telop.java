

package org.firstinspires.ftc.teamcode.outreach;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import java.util.concurrent.TimeUnit;



// any improt that starts with "com.qualcom.robotcore" the link to find it is https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/package-summary.html
@TeleOp(name="Scrappy", group="scrappy")

public class Telop extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    public ElapsedTime runtime = new ElapsedTime();
    //public ElapsedTime time;
    // Drive motors
    public DcMotor LFD;
    public DcMotor LBD;
    public DcMotor RFD;
    public DcMotor RBD;
    
    // Arm and turret
    public DcMotor lift;
    public DcMotor arm;
    public DcMotor turret;
    
    // Servos
    public Servo claw;
    public Servo claw2;
    public Servo claw3;
    public Servo claw4;
    
    // sensors
    //private ColorSensor color;
    //private DigitalChannel touch;
    
    public final static double CLAW_HOME = 0.1;     // Starting position for claw 
    // public static double CLAW_MIN_RANGE = 0.10;  // Smallest number value allowed for claw position
    // public static double CLAW_MAX_RANGE = 0.8;  // Largestnumber value allowed for claw position
    
    
    //public static double clawPosition = CLAW_HOME;  // Sets safe position
    public double CLAW_SPEED;  // Sets rate to move servo
    
    public final static double CLAW2_HOME = 0.1;     // Starting position for claw 
    // public static double CLAW2_MIN_RANGE = 0.10;  // Smallest number value allowed for claw position
    // public static double CLAW2_MAX_RANGE = 0.8;  // Largestnumber value allowed for claw position
    
    //public static double claw2Position = CLAW2_HOME;  // Sets safe position
    public double CLAW2_SPEED; // Sets rate to move servo
    
    public final static double CLAW3_HOME = 0.1;     // Starting position for claw 
    public final static double CLAW3_MIN_RANGE = 0.15;  // Smallest number value allowed for claw position
    public final static double CLAW3_MAX_RANGE = 0.8;  // Largestnumber value allowed for claw position
    
    double claw3Position = CLAW3_HOME;  // Sets safe position
    final double CLAW3_SPEED = 1 ;  // Sets rate to move servo
    
    public final static double CLAW4_HOME = 0.6;     // Starting position for claw 
    public final static double CLAW4_MIN_RANGE = 0.10;  // Smallest number value allowed for claw position
    public final static double CLAW4_MAX_RANGE = 0.7;  // Largestnumber value allowed for claw position
    
    double claw4Position = CLAW4_HOME;  // Sets safe position
    final double CLAW4_SPEED = 1 ;  // Sets rate to move servo
    
    public void RunToPosition() {
        RFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void armRuntoPosition() {
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void turretRuntoPosition() {
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void liftRuntoPosition() {
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void DriveRunToPosition() {
        RFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void ResetEncoders() {
        LFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void DriveResetEncoders() {
        LFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void ResetArmEncoder() {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void ResetTurretEncoder() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void ResetLiftEncoder() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    // Send calculated power to motors
    public void Power(double Power) {
        //turret.setPower(turretPower);
        //arm.setPower(Power);
        LFD.setPower(Power);
        RFD.setPower(Power);
        LBD.setPower(Power);
        RBD.setPower(Power);
    }
    public void Upper(int Pos1, int Pos2, int Pos3) {
        arm.setTargetPosition(Pos1);
        turret.setTargetPosition(Pos2);
        lift.setTargetPosition(Pos3);
    }
    public void Arm(int Pos5) {
        arm.setTargetPosition(Pos5);
    }
    public void Turret(int Pos5) {
        turret.setTargetPosition(Pos5);
    }
    public void Lift(int Pos5) {
        lift.setTargetPosition(Pos5);
    }
    public void TelemetryN() {
        while (arm.isBusy() && LFD.isBusy() && RFD.isBusy() && LBD.isBusy() && RBD.isBusy()) {
            //telemetry.addData("EE, UU");
            telemetry.addData("Arm Encoder", arm.getCurrentPosition());
            telemetry.addData("Arm Power", "%.2f", arm.getPower());
            telemetry.update();
        }
    }
    public void TelemetryL() {
        while (arm.isBusy() && LFD.isBusy() && RFD.isBusy() && LBD.isBusy() && RBD.isBusy()) {
            telemetry.addData("Stauts", "UR MUM");
            telemetry.update();
        }
    }
    public void HoldPos(double Power) {
        arm.setPower(Power);
    }
    public void ArmPower(double Power) {
        arm.setPower(Power);
    }
    public void ArmVelocity(double Power) {
        ((DcMotorEx) arm).setVelocity(Power);
    }
    public void TurretPower(double Power) {
        turret.setPower(Power);
    }
    public void TurretVelocity(double Power) {
        ((DcMotorEx) turret).setVelocity(Power);
    }
    public void LiftPower(double Power) {
        lift.setPower(Power);
    }
    public void LiftVelocity(double Power) {
        ((DcMotorEx) lift).setVelocity(Power);
    }
    public void StopUsingEncoders() {
        LFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void RunUsingEncoders() {
        LFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void armRunUsingEncoders() {
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void ArmStopUsingEncoders() {
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void turretRunUsingEncoders() {
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void turretStopUsingEncoders() {
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void liftRunUsingEncoders() {
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void liftStopUsingEncoders() {
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void DriveRunUsingEncoders() {
        LFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void DriveStopUsingEncoders() {
        LFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    // public void Touch() {
    //     if (touch.getState() == true) {
    //         telemetry.addData("Digital Touch", "Is Not Pressed");
    //     } else {
    //         ArmPower(-1);
    //         telemetry.addData("Digital Touch", "Is Pressed");
    //     }
    // }
    @Override
    public void runOpMode() {
        
        // // touch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        // color = hardwareMap.get(ColorSensor.class, "color");
        LFD  = hardwareMap.get(DcMotor.class, "lfd");
        LBD  = hardwareMap.get(DcMotor.class, "lbd");
        RFD = hardwareMap.get(DcMotor.class, "rfd");
        RBD = hardwareMap.get(DcMotor.class, "rbd");
        turret = hardwareMap.get(DcMotor.class, "turret");
        arm = hardwareMap.get(DcMotor.class, "arm");
        lift = hardwareMap.get(DcMotor.class, "lift");
        claw = hardwareMap.get(Servo.class, "1");
        claw2 = hardwareMap.get(Servo.class, "2");
        claw3 = hardwareMap.get(Servo.class, "3");
        claw4 = hardwareMap.get(Servo.class, "4");
        
        LFD.setDirection(DcMotor.Direction.REVERSE);
        LBD.setDirection(DcMotor.Direction.REVERSE);
        RFD.setDirection(DcMotor.Direction.FORWARD);
        RBD.setDirection(DcMotor.Direction.FORWARD);
        // touch.setMode(DigitalChannel.Mode.INPUT);
        turret.setDirection(DcMotor.Direction.FORWARD);
        
        // LFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // LBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // RFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // RBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveStopUsingEncoders();
        armRunUsingEncoders();
        turretRunUsingEncoders();
        liftRunUsingEncoders();
        
        
        claw.setPosition(CLAW_HOME);
        claw2.setPosition(CLAW2_HOME);
        claw3.setPosition(CLAW3_HOME);
        claw4.setPosition(CLAW4_HOME);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        
        ResetEncoders();
        
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // if (gamepad1.b) {
            //     clawPosition += CLAW_SPEED;
            //     claw2Position -= CLAW2_MIN_RANGE;
            // }else if (gamepad1.y) {
            //     clawPosition -= CLAW_SPEED;
            //     claw2Position += CLAW2_MAX_RANGE;
            // }
            // else 
            if (gamepad2.x) {
                claw3Position += CLAW3_SPEED;
                claw4Position += CLAW4_MIN_RANGE;
            }else if (gamepad2.a) {
                claw3Position -= CLAW3_SPEED;
                claw4Position -= CLAW4_MAX_RANGE;
            }
            
            
            claw3Position = Range.clip(claw3Position, CLAW3_MIN_RANGE, CLAW3_MAX_RANGE);
            claw3.setPosition(claw3Position);
            
            claw4Position = Range.clip(claw4Position, CLAW4_MIN_RANGE, CLAW4_MAX_RANGE);
            claw4.setPosition(claw4Position);
            
            // Display the current value
            
            // if (gamepad2.left_bumper) {
            //     armRunUsingEncoders();
            //     Arm(180);
            //     ArmPower(1);
            //     armRuntoPosition();
            //     // HoldPos(0.05);
            //     TelemetryL();
            //     sleep(10);
            // // } else if (gamepad2.left_bumper = false) {
            // //     // double armPower;
            // // arm.setPower(armPower);
            // //      //ArmStopUsingEncoders();
            // // HoldPos(0);
            // // armPower = (1 * gamepad2.left_stick_y);
            // // // touch();
            // // armPower = (1 * -gamepad2.left_stick_y);
            // // // HoldPos(0.2);
            // // // touch();
            // // Telemetry();
            
            // }
            /*telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();*/

            idle();
            StopUsingEncoders();
            double max;
            
           
            
            // double left   =  -gamepad1.left_trigger; // strafing left
            // double right = (0.7 * -gamepad1.right_trigger); // strafing right
            double drive = -gamepad1.left_stick_y;
            double turn  = -gamepad1.left_stick_x;
            double yaw     =  -gamepad1.left_stick_x; // stafing
            //double turns = gamepad1.right_stick_x;
            //double turnss = -gamepad1.right_stick_x;
            // public void strafe() {
                
            // }
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            
            // double RFPower = drive + turn - left + right;
            // double LFPower = drive - turn + left - right;
            // double LBPower = drive - turn + right - left;
            // double RBPower = drive + turn - right + left;
            
            double RFPower = drive + turn + yaw;
            double LFPower = drive - turn + yaw;
            double LBPower = drive - turn - yaw;
            double RBPower = drive + turn - yaw;
            
            // double drive = -gamepad1.left_stick_y;
            // double turn  = -gamepad1.right_stick_x;
            // LFPower = Range.clip(drive + turn, -1, 1) ;
            // LBPower = Range.clip(drive - turn, -1, 1) ;
            // RFPower = Range.clip(drive + turn, -1, 1) ;
            // RBPower = Range.clip(drive - turn, -1, 1) ;
            
            LFD.setPower(LFPower);
            RFD.setPower(RFPower);
            LBD.setPower(LBPower);
            RBD.setPower(RBPower);
            
            
            //RFPower = Range.clip(turns, -0.5, 0.5) ; 
            //RBPower = Range.clip(turnss, -0.5, 0.5) ; 
            //LFPower = Range.clip(turnss, 0.5, -0.5) ;
            //LBPower = Range.clip(turns, 0.5, -0.5) ;
            
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(LFPower), Math.abs(RFPower));
            max = Math.max(max, Math.abs(LFPower));
            max = Math.max(max, Math.abs(RFPower));
            max = Math.max(max, Math.abs(LBPower));
            max = Math.max(max, Math.abs(RBPower));

            if (max > 1) {
                LFPower = max;
                RFPower = max;
                LBPower = max;
                RBPower = max;
            }
            
            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.
            
            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */
            //double turretPower;
            double armPower;
            double turretPower;
            double liftPower;
            // double clawPosition;
            // double claw2Position;
            // double CLAW_SPEED;
            // double CLAW2_SPEED;
            
            //double drive = gamepad2.right_trigger;
            //double turn  =  -gamepad2.left_stick_y;
            //turretPower    = Range.clip(drive, -1.0, 1.0) ;
            //armPower   = Range.clip(turn, -1, 1);
            //armPower   = Range.clip(drive, -0.7, 0.7);
            
            // if (gamepad2.left_bumper == true) {
            //     armRunUsingEncoders();
            //     Arm(180);
            //     ArmPower(1);
            //     armRuntoPosition();
            //     // HoldPos(0.05);
            //     TelemetryN();
            //     sleep(10);
            // } else if (gamepad2.left_bumper == false) {
                
            // } /*else if (gamepad2.left_bumper) {
            //     RunUsingEncoders();
            //     HoldPos(0);
            //     Arm(2900, 1);
            //     RunToPosition();
            //     Power(0);
            //     TelemetryL();
            //     HoldPos(0.1);
            //     TelemetryL();
            // } else if (gamepad2.b) {
            //     RunUsingEncoders();
            //     HoldPos(0);
            //     Arm(0, 1);
            //     RunToPosition();
            //     Power(0);
            //     TelemetryL();
            //     HoldPos(0.1);
            //     TelemetryL();
            // }*/
            
            ArmStopUsingEncoders();
            turretStopUsingEncoders();
            liftStopUsingEncoders();
            
            // HoldPos(0);
            // armPower = (1 * gamepad1.left_stick_y);
            // turretPower = (1 * gamepad1.right_stick_x);
            // touch();
            armPower = (1 * gamepad2.left_stick_y);
            double clawPosition = (1 * -gamepad2.left_stick_x);
            double claw2Position = (1 * -gamepad2.left_stick_y);
            turretPower = (1 * gamepad2.right_stick_x);
            liftPower = (1 * -gamepad2.right_stick_y);
            // HoldPos(0.2);
            // touch();
            //Telemetry();
            
            
            // clawPosition = Range.clip(clawPosition, CLAW_SPEED);
            claw.setPosition(clawPosition);
            
            
            // claw2Position = Range.clip(claw2Position, CLAW2_MIN_RANGE, CLAW2_MAX_RANGE);
            claw2.setPosition(claw2Position);
            
            max = Math.max(Math.abs(armPower), Math.abs(turretPower));
            max = Math.max(max, Math.abs(armPower));
            max = Math.max(max, Math.abs(turretPower));
            max = Math.max(max, Math.abs(liftPower));

            if (max > 1) {
                armPower = max;
                turretPower = max;
                liftPower = max;
            }
            
            // while (armPower = (1 * -gamepad2.left_stick_y)) {
            //     clawPosition += 1;
            //     claw2Position += 1;
            // }
            
            arm.setPower(armPower);
            turret.setPower(turretPower);
            lift.setPower(liftPower);
            
            // while (armPower = true) {
            //     if (armPower <= 1) {
                    
            //     }
            // }
            
            
            /*LFPosition && RFPosition && LBPosition && RBPosition = (1 * -gamepad2.left_stick_y);
            
            LFPosition && RFPosition && LBPosition && RBPosition = (1 * -gamepad2.right_stick_x);*/
            
            
            /*LFD.setTargetPosition(LFPosition);
            RFD.setTargetPosition(RFPosition);
            LBD.setTargetPosition(LBPosition);
            RBD.setTargetPosition(RBPosition);
            arm.setTargetPosition(armPosition);*/
            
            

            //double time;
            //Telemetry();
            // Show the elapsed game time and wheel power.
            
            
                telemetry.addData("LF Encoder:", LFD.getCurrentPosition());
                telemetry.addData("LF Power ", "%.2f", LFPower);
                telemetry.addData("RF Encoder:", RFD.getCurrentPosition());
                telemetry.addData("RF Power", "%.2f", RFPower);
                telemetry.addData("LB Encoder:", LBD.getCurrentPosition());
                telemetry.addData("LB Power", "%.2f", LBPower);
                telemetry.addData("RB Encoder:", RBD.getCurrentPosition());
                telemetry.addData("RB Power", "%.2f", RBPower);
                telemetry.addData("Arm Encoder", arm.getCurrentPosition());
                telemetry.addData("Arm Power", "%.2f", armPower);
                telemetry.addData("turret Encoder", turret.getCurrentPosition());
                telemetry.addData("turret Power", "%.2f", turretPower);
                telemetry.addData("lift Encoder", lift.getCurrentPosition());
                telemetry.addData("lift Power", "%.2f", liftPower);
                /*telemetry.addData("Left/Right", "%4.2f",  "%4.2f", LFPower, RFPower);
                telemetry.addData("Left/Right", "%4.2f",  "%4.2f", LBPower, RBPower);*/
                telemetry.addData("Status", "Run Time: " + runtime.time(TimeUnit.MINUTES) + runtime.time(TimeUnit.SECONDS));
                //telemetry.addData("Color", "%1.0f", color.argb());
                //telemetry.addData("color", color.getrgba);
                telemetry.update();
                /*telemetry.addData("LF-Encoder/Power", "%4.2f", LFD.getCurrentPosition(), LFPower);
                telemetry.addData("RF-Encoder/Power", "%4.2f", RFD.getCurrentPosition(), RFPower);
                telemetry.addData("LB-Encoder/Power", "%4.2f", LBD.getCurrentPosition(), LBPower);
                telemetry.addData("RB-Encoder/Power", "%4.2f", RBD.getCurrentPosition(), RBPower);
                telemetry.addData("Arm-Encoder/Power", "%4.2f", arm.getCurrentPosition(), armPower);
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.update();*/
                /*telemetry.addData("Front left/Right", "%4.2f, %4.2f", LFPower, RFPower);
                telemetry.addData("Back  left/Right", "%4.2f, %4.2f", LBPower, RBPower);
                telemetry.addData("Motors", "arm (%.2f)", armPower);
                telemetry.update();*/
        }
    }}


