package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleStrafeWREV", group="Official")
/*Log for "OfficalTeleOpWRevServo"
    FTC - TecTigers 10382
    TeleOP
    TeleOp Code for competition with REV robotics smart servo instead of a 
    motor for the upper arm
    Edit log: 
    A. Reyes on 12/5/18
       Added deadzones to arm controls
    A. Reyes on 12/5/18
        added deadzones to drive code to prevent possible coninuous movement like 
            what happened with the arm motors
        added latching servo
        added encoders and encoder telemetry
    A. Reyes on 12/8/18
        removed telemetry values
        added speedCut telemetry to the top
    A. Reyes on 12/14/18
        replaced upper arm motor with CRServo
        Programmed for VEX 393
        added more comments for code
    A. Reyes on 1/3/19
        replaced upper arm servo with DcMotor
        changed variable upperArmServo to upperArmMotor
*/
public class TeleStrafeWREV extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    
    //declares all motors
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, rightLowerArm, leftLowerArm,
        upperArmMotor;
    
    //declares all servos
    private CRServo leftIntakeServo, rightIntakeServo, hangingServo;
    //private Servo mineralServo;
    
    //CRServo values to operate
    private double stop = .02;
    private double back = -1.0;
    private double forward = 1.0;
    
    //Variable to determine speedCut
    private boolean speedCut = false;
    
    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        
        //initializes motors
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class,"right_back_drive");
        rightLowerArm = hardwareMap.get(DcMotor.class,"right_lower_arm");
        leftLowerArm = hardwareMap.get(DcMotor.class, "left_lower_arm" );
        upperArmMotor = hardwareMap.get(DcMotor.class, "upper_arm_motor");
        
        //resets encoders at zero
        rightLowerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLowerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        //starts encoders
        rightLowerArm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        leftLowerArm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        
        //VEX 393 motors initialization (programmed as CRServo)
        rightIntakeServo = hardwareMap.get(CRServo.class,"right_intake_servo"); 
        leftIntakeServo = hardwareMap.get(CRServo.class,"left_intake_servo"); 
        
        //iniializes CRServos
        hangingServo = hardwareMap.get(CRServo.class,"hanging_servo");
        
        
        //initialize servos
        //mineralServo = hardwareMap.get(Servo.class, "mineral_servo");
    
        //sets direction of motors
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
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
    
        //set inital speeds of all Continuous Rotation servos
        hangingServo.setPower(stop);
        //upperArmServo.setPower(stop);
         
        //VEX motors
        leftIntakeServo.setPower(0.0); 
        rightIntakeServo.setPower(0.0);
       
        //initial position of servo
        //mineralServo.setPosition(0.0);
        
        telemetry.addData("Status", "Initialized");
    }

    /* Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY*/
    @Override
    public void init_loop() {
    }

    /*Code to run ONCE when the driver hits PLAY*/
    @Override
    public void start() {
        runtime.reset();
    }

    /*Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP*/
    @Override
    public void loop() {
        //driving using tank drive
        if(gamepad1.dpad_down){
            speedCut = true;    
        }else if(gamepad1.dpad_up){
            speedCut = false;
        }
        
        //speed of the motors
        //drive code 
        rightFrontDrive.setPower((gamepad1.right_stick_y + gamepad1.right_stick_x)/2);
        rightBackDrive.setPower((gamepad1.right_stick_y - gamepad1.right_stick_x)/2);
        leftFrontDrive.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x)/2);
        leftBackDrive.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x)/2);
        
        //Raise and lower arm
        if(gamepad2.left_stick_y > .05 || gamepad2.left_stick_y < -.05){
            leftLowerArm.setPower(gamepad2.left_stick_y);
            rightLowerArm.setPower(-gamepad2.left_stick_y); //made negative to fix
        }else{
            leftLowerArm.setPower(0);
            rightLowerArm.setPower(0);
        }
        
        //raise and lower intake
        if(gamepad2.right_stick_y > .05){
            upperArmMotor.setPower(gamepad2.right_stick_y );
        }else if(gamepad2.right_stick_y < -.05){
            upperArmMotor.setPower(gamepad2.right_stick_y );
        }else{
            upperArmMotor.setPower(0);
        }
       
        //Intake mechanism
        if(gamepad2.right_trigger != 0){
            leftIntakeServo.setPower(-1.0);
            rightIntakeServo.setPower(1.0);
        }else if(gamepad2.left_trigger != 0){
            leftIntakeServo.setPower(1.0);
            rightIntakeServo.setPower(-1.0);
        }else{
            leftIntakeServo.setPower(0.0);
            rightIntakeServo.setPower(0.0);
        }
        
        //hanging manipulator
        if(gamepad2.x){
            hangingServo.setPower(forward);
        }else if(gamepad2.y){
            hangingServo.setPower(back);
        }else{
            hangingServo.setPower(stop);
        }
        
        //mineral manipulator
        /*if(gamepad1.a){
            mineralServo.setPosition(1.0);
        }else if(gamepad1.b){
            mineralServo.setPosition(0.0);
        }*/
        
        //Add data to the phones
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addLine("VALUES");
        telemetry.addData("speedCut", speedCut);
        telemetry.addData("LeftArmEncoderValue:", leftLowerArm.getCurrentPosition());
        telemetry.addData("RightArmEncoderValue:", rightLowerArm.getCurrentPosition());
        telemetry.addLine("");
        
        telemetry.addLine("MOTORS & SERVOS");
        telemetry.addData("rightIntakeServo", rightIntakeServo.getPower());
        telemetry.addData("leftIntakeServo", leftIntakeServo.getPower());
        telemetry.addData("hangingServo", hangingServo.getPower());
        //telemetry.addData("mineralServo position:", mineralServo.getPosition());
        //telemetry.addLine("");
        
        /*telemetry.addLine("CONTROLLERS");
        telemetry.addData("gamepad2.x - ",gamepad2.x);
        telemetry.addData("gamepad2.y - ",gamepad2.y);
        telemetry.addData("gamepad2.right_trigger - ", gamepad2.right_trigger);
        telemetry.addData("gamepad2.left_trigger - ", gamepad2.left_trigger);*/
    }

    /*Code to run ONCE after the driver hits STOP*/
    @Override
    public void stop() {
    }

}
