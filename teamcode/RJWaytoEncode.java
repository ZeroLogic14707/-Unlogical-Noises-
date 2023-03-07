package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous ( name = "someting wong")

public class RJWaytoEncode extends LinearOpMode {
    
    private DcMotor LBD;
    private DcMotor LFD;
    private DcMotor RBD;
    private DcMotor RFD;
    private Servo claw2;
    private Servo claw;
    private DcMotor arm;
    private ColorSensor color;
    
    
    public void ResetEncodersDT(){
    LBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
}
    
    public void RunPosDT(){
        LBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    
    public void SetTgPosDT(int Position1, int Position2){
        LFD.setTargetPosition(Position1);
        RFD.setTargetPosition(Position2);
        LBD.setTargetPosition(Position1);
        RBD.setTargetPosition(Position2);
        
    }
    
    
    public void SetTgPosStrafeDT(int Position1, int Position2, int Position3, int Position4){
        LFD.setTargetPosition(Position1);
        RFD.setTargetPosition(Position2);
        LBD.setTargetPosition(Position3);
        RBD.setTargetPosition(Position4);
    }
    
    public void SetVelocityDT(int Power){
        ((DcMotorEx) LFD).setVelocity(Power);
        ((DcMotorEx) RFD).setVelocity(Power);
        ((DcMotorEx) LBD).setVelocity(Power);
        ((DcMotorEx) RBD).setVelocity(Power);
    }
    
    public void telemetry(){
        while(LBD.isBusy()){
            telemetry.addData("Status", "Waitingfor motors to reach their targets");
            telemetry.update();
            
            
        }
    }
    
    
    public void runOpMode (){
        //naming motors
        RFD = hardwareMap.get(DcMotor.class, "right front drive");
        RBD = hardwareMap.get(DcMotor.class, "right back drive");
        LFD = hardwareMap.get(DcMotor.class, "left front drive");
        LBD = hardwareMap.get(DcMotor.class, "left back drive");
        color = hardwareMap.get(ColorSensor.class, "color");
        
        
        int Red; 
        int Blue;
        int Green;
        
        Red = color.red();
        Blue = color.blue();
        Green = color.green();
        
        ResetEncodersDT();
        
        waitForStart();
        LFD.setDirection(DcMotor.Direction.REVERSE);
        LBD.setDirection(DcMotor.Direction.REVERSE);
        RFD.setDirection(DcMotor.Direction.FORWARD);
        RBD.setDirection(DcMotor.Direction.FORWARD);
        
        {
        ResetEncodersDT();
        SetTgPosDT(1376, 1376);
        RunPosDT();
        SetVelocityDT(1000);
        telemetry();
        SetVelocityDT(0);
        sleep(100);
        
       telemetry.addData("Red", color.red());
       telemetry.addData("Green", color.green());
       telemetry.addData("Blue", color.blue());
       telemetry.update();
}
    // todo: write your code here
    

        if (Red > Green && Red > Blue){
            
            telemetry.addLine("Red");
            telemetry.update();
            sleep(100);
            
         ResetEncodersDT();
            SetTgPosStrafeDT(1376, -1376,-1376, 1376);
            RunPosDT();
            SetVelocityDT(1000);
            telemetry();
            SetVelocityDT(0);
            sleep(100);
        }
       sleep(10000);




}
}