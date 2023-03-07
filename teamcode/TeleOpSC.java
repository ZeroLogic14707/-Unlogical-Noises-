package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp (name = "PowerPlaySC")
public class TeleOpSC extends LinearOpMode{
  
  //makes code happy (names from configuration on driver station)
  public DcMotor BLD;
  public DcMotor BRD;
  public DcMotor FLD;
  public DcMotor FRD;
  
  
  
  @Override
  public void runOpMode() {
    //names motors
    BLD = hardwareMap.get(DcMotor.class, "left back drive");
    BRD = hardwareMap.get(DcMotor.class, "right back drive");
    FLD = hardwareMap.get(DcMotor.class, "left front drive");
    FRD = hardwareMap.get(DcMotor.class, "right front drive");
    
    
    //sets motors to run without encoders
    BLD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); 
    BRD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); 
    FLD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); 
    FRD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); 
    
    
    waitForStart();
    //sets motors to reverse
    BRD.setDirection(DcMotorSimple.Direction.REVERSE);
    FRD.setDirection(DcMotorSimple.Direction.REVERSE);
    
    //sets claw to open at start
    
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        
        //gamepad 1
        //moves drivetrain front/back and turns
        BLD.setPower(1 * gamepad1.left_stick_y);
        BRD.setPower(1 * gamepad1.right_stick_y);
        FLD.setPower(1 * gamepad1.left_stick_y);
        FRD.setPower(1 * gamepad1.right_stick_y);
        
        //strafes right
        BLD.setPower(1 * gamepad1.right_trigger);
        BRD.setPower(-1 * gamepad1.right_trigger);
        FLD.setPower(-1 * gamepad1.right_trigger);
        FRD.setPower(1 * gamepad1.right_trigger);
        
        //strafes left
        BLD.setPower(-1 * gamepad1.left_trigger);
        BRD.setPower(1 * gamepad1.left_trigger);
        FLD.setPower(1 * gamepad1.left_trigger);
        FRD.setPower(-1 * gamepad1.left_trigger);
        
        
      }
    }
  }
}
