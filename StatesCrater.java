package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;


@Autonomous(name = "StatesCrater", group = "Testing")

public class StatesCrater extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, rightLowerArm, leftLowerArm, upperArmMotor;
    
    //declares all servos
    private CRServo leftIntakeServo, rightIntakeServo, hangingServo, extendiBoi1, extendiBoi2;
    private OpticalDistanceSensor HangingODS, leftDriveODS;
    
    double stop = .02;
    double back = -1.0; //in for pin
    double forward = 1.0;
    
    int iterateODS = 0;
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AYWGcTj/////AAABmW8QlFyADE57slQ+RlRh3jhmcaAcY34hi2fx+1MbhOj2tSubwzptbd27EjrpAy3z9w3AoCXWfzUOat4BHvYLvjo0wl78Z47YW1JhzAo5+R3M9QxzX4LyrGn99D67ZxjcnWSukyRsSeIyKW3sPMIDEXfmX3D17OS6otvKFlaHOAvZNMvKeKLG9slQuyv2RLW5YronBwsFJRy1Llo/an/yJ7/2o2XU3OtHL7u5BrEKqLC8PX/pxAfXEUm79/tjcuoP9/CdR+YBuKUZ1G++ki6ZIfPTut3HVI4620Z6MyFSs9n2A4JabHc02/YClitim6yYU0tpWkd+0RuhiN0q4EjS5RDfJZK84NDbrbwhsbpTmbOM";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    
    
    public void movePin(double power, int time){
        hangingServo.setPower(power);
        sleep(time);
        hangingServo.setPower(stop);
        sleep(10);
    }
    
      public void driveForward(double power, int time){
        leftFrontDrive.setPower(-power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(-power);
        rightBackDrive.setPower(power);
        sleep(time);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        sleep(10);
    }   
    
        public void strafeRight(double power, int time){
            rightFrontDrive.setPower(1.00);
            rightBackDrive.setPower(-1.00);
            leftFrontDrive.setPower(1.00);
            leftBackDrive.setPower(-1.00);
            sleep(time);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            sleep(10);
        }
    
        public void strafeLeft(double power, int time){
            rightFrontDrive.setPower(-1.00);
            rightBackDrive.setPower(1.00);
            leftFrontDrive.setPower(-1.00);
            leftBackDrive.setPower(1.00);
            sleep(time);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            sleep(10);
        }
    public void turnLeft(double power, int time){
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
    
    
    public void turnRight(double power, int time){
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
    
    public void extendTheBoys(double power, int time){
        extendiBoi1.setPower(power);
        extendiBoi2.setPower(-power);
        sleep(time);
        extendiBoi1.setPower(0);
        extendiBoi2.setPower(0);
        sleep(10);
    }
 
    
    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        
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
        //rightIntakeServo = hardwareMap.get(Servo.class,"right_intake_servo"); 
        //leftIntakeServo = hardwareMap.get(Servo.class,"left_intake_servo"); 
        
        
        //initializes CRServos
        hangingServo = hardwareMap.get(CRServo.class,"hanging_servo");
        extendiBoi1 = hardwareMap.get(CRServo.class, "extendiBoi1");
        extendiBoi2 = hardwareMap.get(CRServo.class, "extendiBoi2");
        
        //initialize sensors
        HangingODS = hardwareMap.opticalDistanceSensor.get("HangingODS");
        leftDriveODS = hardwareMap.opticalDistanceSensor.get("leftDriveODS");
        
        
        //initialize servos
        //mineralServo = hardwareMap.get(Servo.class, "mineral_servo");
    
        //sets direction of motors
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);//r
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);///r
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
        extendiBoi1.setPower(stop);
        extendiBoi2.setPower(stop);
        //upperArmServo.setPower(stop);
         
        //VEX motors
        leftIntakeServo.setPower(0.0); 
        rightIntakeServo.setPower(0.0);

        //Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        
        waitForStart();

        //extendTheBoys(1.00, 3500);

        //Drop from the lander
        // extend second stagew before dropping from lander
        armSpin(-0.75, 500);
        armSpin(0.75, 1250);
        telemetry.addData("bet", HangingODS.getLightDetected());
        telemetry.update();
        while(iterateODS<10){
            if(HangingODS.getLightDetected() < 0.4){ // Drive to lander until close
                driveForward(-0.75, 10);
                iterateODS++;
            }else{
                iterateODS = 10;
            }
            
        }
        movePin(1.00, 3001); // Hanging rack and pinion OUT
        //strafeRight(1.00, 250);

        armSpin(-0.75, 300);

        //Drive to proper location to scan left two minerals
        driveForward(1.00, 35);
        strafeRight(1.00, 300);
        turnRight(1.00, 40);
        movePin(-1.00, 2500);
        turnRight(1.00, 40);
        driveForward(1.00, 35);

        //Image Recognition
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
            
            //Scan the particles
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                      if (updatedRecognitions.size() == 2) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                          if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                          } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                          } else {
                            silverMineral2X = (int) recognition.getLeft();
                          }
                        }
                      
                      //Process scans and move
                      
                        if (goldMineralX != -1 && silverMineral1X != -1 /*&& silverMineral2X != -1*/) {
                          if (goldMineralX < silverMineral1X) {
                            telemetry.addData("Gold Mineral Position", "Center");
                            //knock off left mineral
                                //turnRight(1.00, 300);
                            
                                //turn left
                                //strafeLeft(1.00, 600);
                                turnLeft(1.00, 100);
                                driveForward(1.00, 1000);
                                driveForward(-1.00, 650);
                                
                                //drive to depot
                                turnLeft(1.00, 550);
                                driveForward(1.00, 1265);
                                turnLeft(1.00, 435);
                                driveForward(1.00, 1550);
                                
                                intakeSpin(1.00, 2000);
                                driveForward(-1.00, 2100);
                                turnLeft(1.00, 1300);
                                extendTheBoys(1.00, 8000);
                            
                          } else if (goldMineralX > silverMineral1X) {
                            telemetry.addData("Gold Mineral Position", "Right");
                            //knock off middle mineral
                                turnRight(1.00, 200);
                            
                                //don't turn
                                driveForward(1.00, 1000);
                                driveForward(-1.00, 450);
                                
                                //drive to depot
                                
                                turnLeft(1.00, 550);
                                driveForward(1.00, 2765);
                                turnLeft(1.00, 180);
                                driveForward(1.00, 1300);
                                
                                intakeSpin(1.00, 2000);
                                driveForward(-1.00, 2700);
                                turnLeft(1.00, 900);
                                extendTheBoys(1.00, 8000);
                          } else{
                            telemetry.addData("Gold Mineral Position", "Unknown");
                            //gold unknown, guess and knock off left mineral
                                turnLeft(1.00, 375);
                            
                                //don't turn
                                driveForward(1.00, 1000);
                                driveForward(-1.00, 450);
                            
                                //drive to depot
                                intakeSpin(1.00, 2000);
                                driveForward(-1.00, 2700);
                                turnLeft(1.00, 900);
                                extendTheBoys(1.00, 8000);
                          }
                        }else{
                            telemetry.addData("Gold Mineral Position", "Left");
                            //knock off right mineral
                                strafeLeft(1.00, 300);
                                turnLeft(1.00, 375);
                            
                                //turn right
                                //strafeRight(1.00, 600);
                                driveForward(1.00, 1000);
                                driveForward(-1.00, 450);
                            
                                //drive to depot
                                turnLeft(1.00, 550);
                                driveForward(1.00, 765);
                                turnLeft(1.00, 180);
                                driveForward(1.00, 1300);
                                
                                intakeSpin(1.00, 2000);
                                driveForward(-1.00, 2700);
                                turnLeft(1.00, 900);
                                extendTheBoys(1.00, 8000);
                        }
                      }
                      telemetry.update();
                    }
                }
            }
        }
        
        
        
        //Deliver the team marker
        intakeSpin(1.00, 4000);
        
        //Park (go to crater and extend the arm)
        driveForward(-1.00, 3000);
        

        if (tfod != null) {
            tfod.shutdown();
        }
        
        while (opModeIsActive()) {
            telemetry.update();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
