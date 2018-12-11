package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import org.firstinspires.ftc.robotcore.external.Func;

@Autonomous(name = "RedAirDepot v2", group = "test")

/*Log for "RedAuton1"
    FTC - TecTigers 10382
    Autonomous
    Autonomous code for when we are on the red side closest to the depot
    Edit log: 
        
*/

public class LandAutonDepot_V2 extends LinearOpMode{
    //declares all motors
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, rightLowerArm, leftLowerArm, upperArmMotor;
    
    //declares all servos
    private CRServo leftIntakeServo, rightIntakeServo, hangingServo;
    
    private Servo latchServo; //mineralServo, 
    
    double stop = .02;
    double back = -1.0; //in for pin
    double forward = 1.0; //out for pin
    
    
    public void driveForward(double power, int time){
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(-power);
        sleep(time);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        sleep(10);
    }   
    
    public void turnLeft(double power, int time){
        leftFrontDrive.setPower(-power);
        rightFrontDrive.setPower(-power);
        leftBackDrive.setPower(-power);
        rightBackDrive.setPower(-power);
        sleep(time);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        sleep(10);
    }
    
    
    public void turnRight(double power, int time){
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        sleep(time);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        sleep(10);
    }
    
    public void intakeSpin(double power, int time){
        leftIntakeServo.setPower(power);
        rightIntakeServo.setPower(-power);
        sleep(time);
        leftIntakeServo.setPower(stop);
        rightIntakeServo.setPower(stop);
        sleep(10);
    }
    
    public void armSpin(double power, int time){
        leftLowerArm.setPower(power);
        rightLowerArm.setPower(-power);
        sleep(time);
        leftLowerArm.setPower(0);
        rightLowerArm.setPower(0);
        sleep(10);
    }
    
    public void wristSpin(double power, int time){
        upperArmMotor.setPower(power);
        sleep(time);
        upperArmMotor.setPower(0);
        sleep(10);
    }
    
    public void wristAndIntakeSpin(double power, int time){
        leftIntakeServo.setPower(power);
        rightIntakeServo.setPower(-power);
        upperArmMotor.setPower(power*0.50);
        sleep(time);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        upperArmMotor.setPower(0);
        sleep(10);
    }
    
    public void driveAndIntakeSpin(double power, int time){
        leftIntakeServo.setPower(power);
        rightIntakeServo.setPower(-power);
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(-power);
        sleep(time);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        sleep(10);
    }
    
    public void movePin(double power, int time){
        hangingServo.setPower(power);
        sleep(time);
        hangingServo.setPower(stop);
        sleep(10);
    }
   /* 
    public void mineralSpin(double position){
        mineralServo.setPosition(position);
        sleep(10);
    }*/
    
    
    
    // todo: write your code here
   // @Override
    public void runOpMode() {
    //Initialize
        //initializes motors
        //leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class,"right_back_drive");
        rightLowerArm = hardwareMap.get(DcMotor.class,"right_lower_arm");
        leftLowerArm = hardwareMap.get(DcMotor.class, "left_lower_arm");
        upperArmMotor = hardwareMap.get(DcMotor.class,"upper_arm_drive");
       
        //iniializes Servos
        rightIntakeServo = hardwareMap.get(CRServo.class,"right_intake_servo"); // <-- Coninuous servo
        leftIntakeServo = hardwareMap.get(CRServo.class,"left_intake_servo"); // <-- Coninuous servo
        hangingServo = hardwareMap.get(CRServo.class, "hanging_servo");
        //mineralServo = hardwareMap.get(Servo.class, "mineral_servo");
        latchServo = hardwareMap.get(Servo.class, "latching_servo");
    
        //sets direction of motors
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightLowerArm.setDirection(DcMotor.Direction.REVERSE);
        leftLowerArm.setDirection(DcMotor.Direction.FORWARD);
        upperArmMotor.setDirection(DcMotor.Direction.FORWARD);
        
        //set initial speeds of motor
        leftFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
    
        rightLowerArm.setPower(0.0);
        leftLowerArm.setPower(0.0);
        upperArmMotor.setPower(0.0);
        
        //set inital speeds of all Continuous Rotation servos
        leftIntakeServo.setPower(0.02); // 0.02 is not moving for continuous servo
        rightIntakeServo.setPower(0.02);
        hangingServo.setPower(0.02);
        
        //initial position of servo
        //mineralServo.setPosition(0.0);
        latchServo.setPosition(0.25);
    
     
    //START    
    waitForStart();
    //Drop from Lander
    armSpin(0.75, 300);
    armSpin(-0.75, 1050);
    driveForward(-1.00, 131);
    movePin(-1.00, 3001); // Pin OUT
    
    //latchServo.setPosition(0.0);
    //armSpin(0.75, 400);
    
    //movePin(1.00, 5000); // Pin IN
    //wristSpin(0.75, 300);
    //armSpin(-0.70, 600);
    
    //Image Recognition (Tensor Flow)
    
    //Knock over the yellow cube
    //mineralSpin(); //<-- find position of "down"
     
    //Deliver the team marker
    driveForward(1.00, 1000);
    turnRight(1.00, 800);
    driveForward(1.00, 3000);
    turnLeft(1.00, 1300);
    driveForward(1.00, 1250);
    wristSpin(-0.50, 900);
    armSpin(0.75, 400);
    
    //Spit the team marker out
    //intakeSpin(-1.00, 1600); // Spit the team marker out <-- Old Code 
    wristAndIntakeSpin(-1.00, 1800);
    
    
    //Fold arm back in, get ready for movement to crater 
    //wristSpin(-0.75, 600);
    //armSpin(-0.75, 600);
    //turnLeft(1.00, 950);
    
    //driveForward(-1.00, 5950); <-- Old Move back method 
    driveAndIntakeSpin(-1.00, 5950);
    
    }
}
