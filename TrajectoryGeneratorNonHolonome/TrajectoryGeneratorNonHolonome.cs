using EventArgsLibrary;
using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Utilities;

namespace TrajectoryGeneratorNonHolonomeNS
{
    public class TrajectoryGeneratorNonHolonome
    {
        int robotId;

        double samplingFreq;

        Location currentLocationRefTerrain;
        Location wayPointLocation;
        Location ghostLocationRefTerrain;
        Location cibleProjetee;

        double accelLineaire, accelAngulaire;
        double vitesseLineaireMax, vitesseAngulaireMax;

        AsservissementPID PID_Position_Lineaire;
        AsservissementPID PID_Position_Angulaire;

        public TrajectoryGeneratorNonHolonome(int id)
        {
            robotId = id;
            InitRobotPosition(0, 0, 0);
            InitPositionPID();

            //Initialisation des vitesse et accélérations souhaitées
            accelLineaire = 0.5; //en m.s-2
            accelAngulaire = 0.5 * Math.PI * 1.0; //en rad.s-2

            vitesseLineaireMax = 1; //en m.s-1               
            vitesseAngulaireMax = 1 * Math.PI * 1.0; //en rad.s-1
        }

        void InitPositionPID()
        {
            PID_Position_Lineaire = new AsservissementPID(20.0, 10.0, 0, 100, 100, 1);
            PID_Position_Angulaire = new AsservissementPID(20.0, 10.0, 0, 5 * Math.PI, 5 * Math.PI, Math.PI);
        }

        public void InitRobotPosition(double x, double y, double theta)
        {
            Location old_currectLocation = currentLocationRefTerrain;
            currentLocationRefTerrain = new Location(x, y, theta, 0, 0, 0);
            wayPointLocation = new Location(x, y, theta, 0, 0, 0);
            ghostLocationRefTerrain = new Location(x, y, theta, 0, 0, 0);
            PIDPositionReset();
        }

        public void OnPhysicalPositionReceived(object sender, LocationArgs e)
        {
            if (robotId == e.RobotId)
            {
                currentLocationRefTerrain = e.Location;
                CalculateGhostPosition();
                PIDPosition();
            }
        }



        double ptCibleprojete = 0;
        double vLinG = 0;
        double vLinGarret = 0;
        double dLinG = 0; //G de ghost
        double darret = 0;
        double thetaCible = 0;
        double thetaEcart = 0;

        public enum ghostState { idle, avance_recule, rotation }
        ghostState state = ghostState.idle;

        void CalculateGhostPosition()
        {
            //A remplir
            //ON EN EST LA

            switch (state)
            {
                case ghostState.idle: //idle = attente
                    accelLineaire = 0;
                    break;

                case ghostState.rotation:
                    thetaCible = Math.Pow(Math.Atan((wayPointLocation.Y - currentLocationRefTerrain.Y) / (wayPointLocation.X - currentLocationRefTerrain.X)), 2);
                    thetaEcart = thetaCible - Toolbox.ModuloByAngle( ghostLocationRefTerrain.Theta, wayPointLocation.Theta); //% = modulo ?!?!!!!!! a verifier !!!

                    if(thetaEcart>0.2)
                    {
                        dLinG = Math.Sqrt(Math.Pow((cibleProjetee.X - ghostLocationRefTerrain.X), 2) + Math.Pow((cibleProjetee.Y - ghostLocationRefTerrain.Y), 2));
                        vLinGarret = vLinG / 2 * dLinG;

                        if (dLinG > ghostLocationRefTerrain.Vtheta * ghostLocationRefTerrain.Vtheta / 2 * accelAngulaire)
                        {
                            ghostLocationRefTerrain.Vtheta += accelLineaire / 50;
                        }
                        else
                        {
                            ghostLocationRefTerrain.Vtheta += -accelLineaire / 50; // -!-   <_>     o-|-o     °_°     o_o   
                        }

                    }



                    break;

                case ghostState.avance_recule:

                    

                    break;

            }
                






            //pt projeté cible (r) = produit scalaire entre vecteur cible et vecteur ghost : theta = theta du ghost
            ptCibleprojete = (wayPointLocation.X * ghostLocationRefTerrain.X + wayPointLocation.Y * ghostLocationRefTerrain.Y)/ Math.Sqrt(Math.Pow((ghostLocationRefTerrain.X), 2) + Math.Pow((ghostLocationRefTerrain.Y), 2));
            cibleProjetee.X = ptCibleprojete * Math.Cos(ghostLocationRefTerrain.Theta);
            cibleProjetee.Y = ptCibleprojete * Math.Sin(ghostLocationRefTerrain.Theta);
            cibleProjetee.Theta = ghostLocationRefTerrain.Theta;



            


            


            //On renvoie la position du ghost pour affichage
            OnGhostLocation(robotId, ghostLocationRefTerrain);
        }


        void PIDPosition()
        {
            //A remplir
            //PD pas PID
            /////

            double vLineaireRobot = 0, vAngulaireRobot = 0;
            /////

            //Si tout c'est bien passé, on envoie les vitesses consigne.
            OnSpeedConsigneToRobot(robotId, (float)vLineaireRobot, (float)vAngulaireRobot);
        }

        void PIDPositionReset()
        {
            if (PID_Position_Angulaire != null && PID_Position_Lineaire != null)
            {
                PID_Position_Lineaire.ResetPID(0);
                PID_Position_Angulaire.ResetPID(0);
            }
        }

        /*************************************** Outgoing Events ************************************/

        public event EventHandler<LocationArgs> OnGhostLocationEvent;
        public virtual void OnGhostLocation(int id, Location loc)
        {
            var handler = OnGhostLocationEvent;
            if (handler != null)
            {
                handler(this, new LocationArgs { RobotId = id, Location = loc });
            }
        }

        public event EventHandler<PolarSpeedArgs> OnSpeedConsigneEvent;
        public virtual void OnSpeedConsigneToRobot(int id, float vLineaire, float vAngulaire)
        {
            var handler = OnSpeedConsigneEvent;
            if (handler != null)
            {
                handler(this, new PolarSpeedArgs { RobotId = id, Vx = vLineaire, Vy = 0, Vtheta = vAngulaire });
            }
        }
    }
} 
