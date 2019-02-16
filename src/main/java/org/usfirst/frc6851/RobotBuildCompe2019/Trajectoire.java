package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
import java.lang.*;
import edu.wpi.first.wpilibj.Notifier;
import com.ctre.phoenix.motion.*;
public class Trajectoire{
    public double[][] points;
    private enum Etats 
    {
        ARRET,
        ACCELERATION,
        MAINTIEN_VITESSE
    }

    private TalonSRX mMoteur;
    private double mBaseDeTemp_sec;
    private double mIncrementVitesse;
    private double mIncrementVitesseActif;
    
    private double mIncrementPositionActif;
    private double mIncrementPosition;
    private double mVitesseMax;
    private Etats mEtatTrajectoire;
    private double mPositionActuelle;
    private double mDestination;
    public Trajectoire(TalonSRX moteur, double positionInit,double vitesseMax, double acceleration, double baseDeTemp_sec)
    {
        mMoteur = moteur;
        mBaseDeTemp_sec = baseDeTemp_sec;
        mVitesseMax = vitesseMax;
        mIncrementVitesse = acceleration *  baseDeTemp_sec;
        mIncrementPosition = mIncrementVitesse *  baseDeTemp_sec;
        mEtatTrajectoire = Etats.ARRET;
        mPositionActuelle = positionInit;
        mDestination = positionInit;
        mIncrementVitesseActif = 0;
        mIncrementPositionActif = 0;
    }
    
    public void SetPosition(double position)
    {
        mDestination = position;
    }

    public void Process()
    {
        switch(mEtatTrajectoire)
        {
            case ARRET:
                if (mDestination != mPositionActuelle)
                {
                    mIncrementVitesseActif = mIncrementVitesse;
                    if (mDestination < mPositionActuelle)
                    {
                        mIncrementVitesseActif *= -1;
                    }
                    mEtatTrajectoire = Etats.ACCELERATION; 
                }
                break;
            case ACCELERATION:
                if (Math.abs(mIncrementPositionActif) >= mIncrementPosition)
                {
                    if (mIncrementPositionActif < 0)
                    {
                        mIncrementPositionActif = -1 * mIncrementPosition;
                    } 
                    else
                    {
                        mIncrementPositionActif = mIncrementPosition;
                    }
                    mIncrementVitesseActif = 0;
                    mEtatTrajectoire = Etats.MAINTIEN_VITESSE;
                }
                break;
            case MAINTIEN_VITESSE:
                break;

        }
        mIncrementPositionActif += mIncrementVitesseActif;
        mPositionActuelle += mIncrementPositionActif;
    }
}