// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc6851.RobotBuildCompe2019.subsystems;


import org.usfirst.frc6851.RobotBuildCompe2019.commands.*;
//import org.usfirst.frc6851.robot.utils.MathUtils;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class DriveTrain extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private Spark moteurAvantDroit;
    private Spark moteurAvantGauche;
    private Spark moteurArriereDroit;
    private Spark moteurArriereGauche;
    private MecanumDrive mecanumDrive;
    private Encoder mAvD;
    private Encoder mAvG;
    private Encoder mArD;
    private Encoder mArG;
 

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public DriveTrain() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        moteurAvantDroit = new Spark(2);
        addChild("MoteurAvantDroit",moteurAvantDroit);
        moteurAvantDroit.setInverted(false);
        
        moteurAvantGauche = new Spark(0);
        addChild("MoteurAvantGauche",moteurAvantGauche);
        moteurAvantGauche.setInverted(false); //true
        
        moteurArriereDroit = new Spark(1);
        addChild("MoteurArriereDroit",moteurArriereDroit);
        moteurArriereDroit.setInverted(false); //true
        
        moteurArriereGauche = new Spark(3);
        addChild("MoteurArriereGauche",moteurArriereGauche);
        moteurArriereGauche.setInverted(false);
        
        mecanumDrive = new MecanumDrive(moteurAvantGauche, moteurArriereGauche,
            moteurAvantDroit, moteurArriereDroit);
        addChild("Mecanum Drive",mecanumDrive);
        mecanumDrive.setSafetyEnabled(true);
        mecanumDrive.setExpiration(0.1);
        mecanumDrive.setMaxOutput(1.0);

        
        mAvD = new Encoder(0, 1, false, EncodingType.k4X);
        addChild("MAvD",mAvD);
        mAvD.setDistancePerPulse(1.0);
        mAvD.setPIDSourceType(PIDSourceType.kDisplacement);
        
        mAvG = new Encoder(2, 3, false, EncodingType.k4X);
        addChild("MAvG",mAvG);
        mAvG.setDistancePerPulse(1.0);
        mAvG.setPIDSourceType(PIDSourceType.kDisplacement);
        
        mArD = new Encoder(4, 5, false, EncodingType.k4X);
        addChild("MArD",mArD);
        mArD.setDistancePerPulse(1.0);
        mArD.setPIDSourceType(PIDSourceType.kDisplacement);
        
        mArG = new Encoder(6, 7, false, EncodingType.k4X);
        addChild("MArG",mArG);
        mArG.setDistancePerPulse(1.0);
        mArG.setPIDSourceType(PIDSourceType.kDisplacement);
        

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }
//   { } 
    public void DriveMecanum(Joystick manette){
        
        //mecanumDrive.driveCartesian(manette.getY(), manette.getX(), -manette.getTwist());
        mecanumDrive.driveCartesian(
            (manette.getX() * manette.getX() * manette.getX()),
           -(manette.getY() * manette.getY() * manette.getY()),
            manette.getTwist() * manette.getTwist() * manette.getTwist() 
        );

        mecanumDrive.setMaxOutput(((manette.getRawAxis(3)-1)*-1)/4+0.5); // tentative de lire le Slider

        SmartDashboard.putNumber("encodeurMArD", mArD.getRaw());
        SmartDashboard.putNumber("encodeurMArG", mArG.getRaw());
        SmartDashboard.putNumber("encodeurMAvD", mAvD.getRaw());
        SmartDashboard.putNumber("encodeurMAvG", mAvG.getRaw());
    }   

    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        setDefaultCommand(new Drivewithjoystick(0, 0));

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop

    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}

