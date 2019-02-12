package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.Notifier;
import com.ctre.phoenix.motion.*;
public class Trajectoire{
    public double[][] points;
    public Trajectoire(double posDepart, double posFin, double vitesse, double acceleration)
    {
        double DureeStable;
        double DureeAcceleration;
        double DureeTotale;
        double distance = posFin - posDepart;
        DureeStable = (distance - vitesse) / vitesse;
        DureeAcceleration = vitesse / acceleration;
        DureeTotale = 2 * DureeAcceleration +  DureeStable;
        int nbPoints = (int)DureeTotale * 100;
        points = new double[nbPoints][3];
        
    }
}