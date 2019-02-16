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
    private MotionProfileStatus mMotionStatus = new MotionProfileStatus();
    private double mBaseDeTemp_sec;
    private double mVitesseActuelle;
    private double mIncrementVitesse;
    private double mIncrementVitesseActif;
    private double mIncrementPositionCarre;
    private double mIncrementPositionCarreActif;
    
    
    private double mIncrementPositionActif;
    private double mIncrementPosition;
    private double mVitesseMax;
    private Etats mEtatTrajectoire;
    private double mPositionActuelle;
    private double mDestination;
    private Boolean mFirstPos;
    class PeriodicRunnable implements java.lang.Runnable {
	    public void run() {  mMoteur.processMotionProfileBuffer();    }
	}
	private Notifier mNotifer;

    public Trajectoire(TalonSRX moteur, double positionInit,double vitesseMax, double acceleration, double baseDeTemp_sec)
    {
        mMoteur = moteur;
        mMoteur.changeMotionControlFramePeriod(5);
        mNotifer = new Notifier(new PeriodicRunnable());
        mNotifer.startPeriodic(0.005);
        mBaseDeTemp_sec = baseDeTemp_sec;
        mVitesseMax = vitesseMax;
        mIncrementVitesse = acceleration *  baseDeTemp_sec;
        mIncrementPositionCarre = mIncrementVitesse *  baseDeTemp_sec;
        mEtatTrajectoire = Etats.ARRET;
        mPositionActuelle = positionInit;
        mDestination = positionInit;
        mFirstPos = true;
        mIncrementVitesseActif = 0;
        mIncrementPositionActif = 0;
        mVitesseActuelle = 0;
        mIncrementPositionCarreActif = 0;
    
    }
    public void Reset()
    {
        mMoteur.clearMotionProfileTrajectories();
    }
    public void SetPosition(double position)
    {
        mDestination = position;
    }

    public void MiseAJourDOnnees()
    {
        switch(mEtatTrajectoire)
        {
            case MAINTIEN_VITESSE:
            case ARRET:
                if (mDestination != mPositionActuelle)
                {
                    mIncrementPositionCarreActif = mIncrementPositionCarre;
                    mIncrementVitesseActif = mIncrementVitesse;
                    if (mDestination < mPositionActuelle)
                    {
                        mIncrementPositionCarreActif *= -1;
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
                    mIncrementPositionCarreActif = 0;
                    mEtatTrajectoire = Etats.MAINTIEN_VITESSE;
                }
                break;

        }
        mVitesseActuelle += mIncrementVitesseActif;
        mIncrementPositionActif += mIncrementPositionCarreActif;
        mPositionActuelle += mIncrementPositionActif;
        TrajectoryPoint point = new TrajectoryPoint();
        point.position = mPositionActuelle;
        point.velocity = mVitesseActuelle;
        point.headingDeg  = 0;
        point.profileSlotSelect0 = 0;
        point.timeDur = 10;
        point.zeroPos = mFirstPos;
        if (mFirstPos)
        {
            mFirstPos = false;
        }
        point.isLastPoint = false;
        mMoteur.pushMotionProfileTrajectory(point);
    }

    public void Process()
    {
        mMoteur.getMotionProfileStatus(mMotionStatus);
        if (mMoteur.getControlMode() == ControlMode.MotionProfile) 
        {
            MiseAJourDOnnees();
        }
        
    }
}