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

@TeleOp(name="Official TeleOp", group="Official")
@Disabled
/*Log for "Offial TeleOp"
    FTC - TecTigers 10382
    TeleOP
    Official TeleOp Code for competition
    Edit log: 
        A. Reyes on 12/5/18
            Added deadzones to arm controls
        A. Reyes on 12/5/18
            added deadzones to drive code to prevent possible coninuous movement like 
                what happened with the arm motors
            added latching servo
            added encoders and encoder telemetry
        A. Reyes on 12/7/18
            disabled Code
*/

public class OfficialTele extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    
    //declares all motors
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, rightLowerArm, leftLowerArm, upperArmMotor;
    
    //declares all servos
    private CRServo leftIntakeServo, rightIntakeServo, hangingServo;
    
    private Servo /*mineralServo,*/ latchingServo;
    
    double stop = .02;
    double back = -1.0;
    double forward = 1.0;
    
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
        upperArmMotor = hardwareMap.get(DcMotor.class,"upper_arm_drive");
       
        //resets encoders at zero
        rightLowerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLowerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        //starts encoders
        rightLowerArm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        leftLowerArm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        
       
        //iniializes Servos
        rightIntakeServo = hardwareMap.get(CRServo.class,"right_intake_servo"); // <-- Coninuous servo
        leftIntakeServo = hardwareMap.get(CRServo.class,"left_intake_servo"); // <-- Coninuous servo
        hangingServo = hardwareMap.get(CRServo.class,"hanging_servo");
        
        latchingServo = hardwareMap.get(Servo.class, "latching_servo" );
       // mineralServo = hardwareMap.get(Servo.class, "mineral_servo");
        //test = hardwareMap.get(CRServo.class, "test");
        
      
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
        rightIntakeServo.setPower(stop);
        hangingServo.setPower(0.02);
        
        //revZero = right intake
       
        //initial position of servo
       // mineralServo.setPosition(0.0);
        latchingServo.setPosition(0.0);
        
        
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //driving using tank drive
        /*leftFrontDrive.setPower(gamepad1.left_stick_y);
        leftBackDrive.setPower(gamepad1.left_stick_y);//was neg*/
        if(gamepad1.left_stick_y > .05 || gamepad1.left_stick_y < -.05){
            leftFrontDrive.setPower(gamepad1.left_stick_y);
            leftBackDrive.setPower(gamepad1.left_stick_y);
            
            rightFrontDrive.setPower(-gamepad1.right_stick_y); //made negative to fix
            rightBackDrive.setPower(-gamepad1.right_stick_y);
        }else{
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            
            rightFrontDrive.setPower(0); //made negative to fix
            rightBackDrive.setPower(0);
        }
        
        /*rightFrontDrive.setPower(-gamepad1.right_stick_y);//was neg
        rightBackDrive.setPower(-gamepad1.right_stick_y);*/
        
        //Raise and lower arm
        if(gamepad2.left_stick_y > .05 || gamepad2.left_stick_y < -.05){
            leftLowerArm.setPower(gamepad2.left_stick_y);
            rightLowerArm.setPower(-gamepad2.left_stick_y); //made negative to fix
        }else{
            leftLowerArm.setPower(0);
            rightLowerArm.setPower(0);
        }
        
        //raise and lower intake
        if(gamepad2.right_stick_y > .05 || gamepad2.right_stick_y < -.05){
            upperArmMotor.setPower(gamepad2.right_stick_y*.50);
        }else{
            upperArmMotor.setPower(0.0);
        }
       
        //Intake mechanism
        if(gamepad2.right_trigger != 0){
            leftIntakeServo.setPower(back);
            rightIntakeServo.setPower(forward);
        }else if(gamepad2.left_trigger != 0){
            leftIntakeServo.setPower(forward);
            rightIntakeServo.setPower(back);
        }else{
            leftIntakeServo.setPower(stop);
            rightIntakeServo.setPower(stop);
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
        
        //latching manipulator
        if(gamepad1.x){
            latchingServo.setPosition(1.0);
        }else if(gamepad1.y){
            latchingServo.setPosition(0);
        }
        
        //Add data to the phones
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        
        telemetry.addLine("CONTROLLERS");
        telemetry.addData("gamepad2.x - ",gamepad2.x);
        telemetry.addData("gamepad2.y - ",gamepad2.y);
        telemetry.addData("gamepad2.right_trigger - ", gamepad2.right_trigger);
        telemetry.addData("gamepad2.left_trigger - ", gamepad2.left_trigger);
        
        telemetry.addLine(" ");
        
        telemetry.addLine("MOTORS & SERVOS");
        telemetry.addData("rightIntakeServo", rightIntakeServo.getPower());
        telemetry.addData("leftIntakeServo", leftIntakeServo.getPower());
        telemetry.addData("hangingServo", hangingServo.getPower());
        
        
        
        //telemetry.addData("mineralServo position:", mineralServo.getPosition());
        telemetry.addData("LeftArmEncoderValue:", leftLowerArm.getCurrentPosition());
        telemetry.addData("RightArmEncoderValue:", rightLowerArm.getCurrentPosition());
        //telemetry.addData("");
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
