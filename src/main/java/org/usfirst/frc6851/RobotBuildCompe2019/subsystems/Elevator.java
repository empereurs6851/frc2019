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
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class Elevator extends Subsystem {
    public static final double kdCountPerTurn = 4096.0;
    //public static final double kdInchPerTurn = 6.3;
    public static final double kdInchPerTurn = 10.9;
    //public static final double kdTickPerInch = kdCountPerTurn / kdInchPerTurn ;
    public static final double kdTickPerInch = 1;
    
    public static final double[][] kPositionEtages=
/*
    {
        
        {0.0, 0.0},
        {5.0, 5.0},
        {10.0, 10.0},
        {15.0, 15.0},
        {20.0, 20.0},
        {25.0, 25.0},
        {30.0, 30.0},
        {35.0, 35.0}
        
    }; 
*/
    {
       {0.0, 0.0},
    //   {3500, 3500}, // optimiser deplacements step 1
       {0, 6500}, // maximiser pourche
       //{16700, 0},
//       {8850, 8850}, // optimiser deplacements step 1
       {0, 1670}, // maximiser pourche
       //{0.0, 25530},
//       {12765, 12765}, // optimiser deplacements step 1
       {0, 25530}, // maximiser pourcue
       //{22222, 0},
//       {10611, 10611}, // optimiser deplacements step 1
       {0, 22222}, // maximiser pourche
       //{19300,18700 },
        {18500,18500 }, // optimiser deplacements step 1
        //{19350, 25385},
        {22367, 22367}, // optimiser deplacements step 1
        //{26930,26700 }
        {25149,25149 } // optimiser deplacements step 1
    }; 

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private TalonSRX ascenceur1;
    private TalonSRX ascenceurPourche;
    private DigitalInput limitSwitch1;
    private DigitalInput limitSwitch2;
    private Relay tombePourche;
    private TalonSRX Elevateur1;
    private int EtageCourant;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public Elevator() {
        
        // TALONSRX TEST
        //TalonSRX mytalon = new TalonSRX(4);
        //mytalon.set(ControlMode.PercentOutput,0);
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        //double value = kPositionEtages[0][6];
        ascenceur1 = new TalonSRX(5);
        ascenceur1.selectProfileSlot(0,0);
        ascenceur1.setInverted(false);
        //addChild("Ascenceur1",ascenceur1);
        //ascenceur1.setInverted(false);
        //ascenceur1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
         ascenceur1.set(ControlMode.PercentOutput,0);
        ascenceur1.getSensorCollection().setQuadraturePosition(0,100);
        ascenceur1.set(ControlMode.Position,0);    
        //ascenceur1.set(ControlMode.Position,26624);
        
        ascenceurPourche = new TalonSRX(4);
        ascenceurPourche.selectProfileSlot(0,0);
        ascenceurPourche.set(ControlMode.Position,0);    
        ascenceurPourche.set(ControlMode.PercentOutput,0.1);
        ascenceurPourche.setInverted(false);
        //addChild("AscenceurPourche",ascenceurPourche);
        //ascenceurPourche.setInverted(false);
        ascenceurPourche.set(ControlMode.PercentOutput,0);
        ascenceurPourche.getSensorCollection().setQuadraturePosition(0,100);
        ascenceurPourche.set(ControlMode.Position,0);    
        
        limitSwitch1 = new DigitalInput(8);
        addChild("Limit Switch 1",limitSwitch1);
        
        
        limitSwitch2 = new DigitalInput(9);
        addChild("Limit Switch 2",limitSwitch2);

        
        tombePourche = new Relay(0);
        addChild("TombePourche",tombePourche);
        
        EtageCourant = 0;
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

//        SmartDashboard.putData("Etage 0", new GotoEtage(0));
//        SmartDashboard.putData("Etage 1", new GotoEtage(1));
//        SmartDashboard.putData("Etage 2", new GotoEtage(2));
//        SmartDashboard.putData("Etage 3", new GotoEtage(3));
//        SmartDashboard.putData("Etage 4", new GotoEtage(4));
//        SmartDashboard.putData("Etage 5", new GotoEtage(5));
//        SmartDashboard.putData("Etage 6", new GotoEtage(6));
//        SmartDashboard.putData("Etage 7", new GotoEtage(7));

    }

    public void GoToInch(TalonSRX moteur, double dInchPos)
    {
        moteur.set(ControlMode.Position,kdTickPerInch * dInchPos);
        SmartDashboard.putNumber("Ascenceur - Cible", kdTickPerInch * dInchPos);
    }
    public void GotoEtage(int etage)
    {
        if (EtageCourant<etage)
        {
            ascenceur1.selectProfileSlot(1,0); // sLOT 1 POUR MONTER
            ascenceurPourche.selectProfileSlot(1,0);
        }
        else
        {
            ascenceur1.selectProfileSlot(0,0); // SLOT 0 POUR DESCRNDRE
            ascenceurPourche.selectProfileSlot(0,0);
        }
        EtageCourant = etage;
        if (EtageCourant > 7 )
        {
            EtageCourant= 7 ;
        }
        if (EtageCourant< 0)
        {
            EtageCourant = 0;
        }
        GoToInch(ascenceur1, kPositionEtages[EtageCourant][0]);
        GoToInch(ascenceurPourche, kPositionEtages[EtageCourant][1]);
        SmartDashboard.putNumber("Ascenceur1 - Cible", kPositionEtages[EtageCourant][0]);
        SmartDashboard.putNumber("AscenceurPourche - Cible", kPositionEtages[EtageCourant][1]);
    }
    public void EtageMonte()
    {
        EtageCourant++;
        GotoEtage(EtageCourant);
    }

    public void ManPourcheMonte()
    {
        if (ascenceurPourche.getSensorCollection().getQuadraturePosition()>25000){
            GoToInch(ascenceurPourche,26000);
        } else {
            GoToInch(ascenceurPourche,1000+ascenceurPourche.getSensorCollection().getQuadraturePosition());
        }    
    }
    public void ManPourcheDescend()
    {
        if (ascenceurPourche.getSensorCollection().getQuadraturePosition()<1000){
            GoToInch(ascenceurPourche,0);
        } else {
            GoToInch(ascenceurPourche,-1000+ascenceurPourche.getSensorCollection().getQuadraturePosition());
        }    
    }
    public void Man1Monte()
    {
        if (ascenceur1.getSensorCollection().getQuadraturePosition()>25000){
            GoToInch(ascenceur1,26000);
        } else {
            GoToInch(ascenceur1,1000+ascenceur1.getSensorCollection().getQuadraturePosition());
        }    
    }
    public void Man1Descend()
    {
        if (ascenceur1.getSensorCollection().getQuadraturePosition()<1000){
            GoToInch(ascenceur1,0);
        } else {
            GoToInch(ascenceur1,-1000+ascenceur1.getSensorCollection().getQuadraturePosition());
        }    
    }

    public void EtageDescend()
    {
        EtageCourant--;
        GotoEtage(EtageCourant);
    }


    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop
        SmartDashboard.putNumber("Ascenceur 1       - Position", ascenceur1.getSensorCollection().getQuadraturePosition());
        SmartDashboard.putNumber("Ascenceur Pourche - Position", ascenceurPourche.getSensorCollection().getQuadraturePosition());
//        double valeur = SmartDashboard.getNumber("ALLO", 26624);
//        ascenceur1.set(ControlMode.Position,valeur);
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}

