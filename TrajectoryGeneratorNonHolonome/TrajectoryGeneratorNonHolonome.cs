﻿using EventArgsLibrary;
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
        double toleranceAng;
        double tolerancedist;

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

            toleranceAng = 0.2;
            tolerancedist = 0.2;
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




        double thetaArret = 0;
        double thetaEcart = 0;
        double distArret = 0;
        double distEcart = 0;
        double ptCibleprojete = 0;
        double distRobotCibleProjeteeX = 0;
        double distRobotCibleProjeteeY = 0;
        double distRobotCibleProjetee = 0;
        double angleGhostWaypoint = 0;


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
                    
                    thetaEcart = wayPointLocation.Theta - Toolbox.ModuloByAngle(wayPointLocation.Theta, currentLocationRefTerrain.Theta);
                    thetaArret = Math.Pow(ghostLocationRefTerrain.Vtheta, 2) / 2 * accelAngulaire;

                    if (thetaEcart > 0)
                    {
                        if (ghostLocationRefTerrain.Vtheta < 0)
                        {
                            ghostLocationRefTerrain.Vtheta -= accelAngulaire / 50;
                        }
                        else
                        {
                            if (thetaEcart > thetaArret)
                            {
                                if (ghostLocationRefTerrain.Vtheta < vitesseAngulaireMax)
                                {
                                    ghostLocationRefTerrain.Vtheta += accelAngulaire / 50; // -!-   <_>     o-|-o     °_°     o_o 
                                }
                                else
                                {
                                    ghostLocationRefTerrain.Vtheta = ghostLocationRefTerrain.Vtheta;
                                }
                            }
                            else
                            {
                                ghostLocationRefTerrain.Vtheta -= accelAngulaire / 50;
                            }  
                        }
                    }
                    else
                    {
                        if (ghostLocationRefTerrain.Vtheta > 0)
                        {
                            ghostLocationRefTerrain.Vtheta += accelAngulaire / 50;
                        }
                        else
                        {
                            if (Math.Abs(thetaEcart) > thetaArret)
                            {
                                if (ghostLocationRefTerrain.Vtheta > -vitesseAngulaireMax)
                                {
                                    ghostLocationRefTerrain.Vtheta -= accelAngulaire / 50; // -!-   <_>     o-|-o     °_°     o_o 
                                }
                                else
                                {
                                    ghostLocationRefTerrain.Vtheta = ghostLocationRefTerrain.Vtheta;
                                }
                            }
                            else
                            {
                                ghostLocationRefTerrain.Vtheta += accelAngulaire / 50;
                            }
                        }
                    }
                    if (Math.Abs(thetaEcart) < toleranceAng)
                    {
                        state = ghostState.avance_recule;
                    }
                    break;


                case ghostState.avance_recule:
                    
                    distEcart = wayPointLocation.X - Toolbox.ModuloByAngle(wayPointLocation.X, currentLocationRefTerrain.X);
                    distArret = Math.Pow(ghostLocationRefTerrain.Vx, 2) / 2 * accelLineaire;

                    if (distEcart > 0)
                    {
                        if (ghostLocationRefTerrain.Vx < 0)
                        {
                            ghostLocationRefTerrain.Vx -= accelLineaire / 50;
                        }
                        else
                        {
                            if (distEcart > distArret)
                            {
                                if (ghostLocationRefTerrain.Vx < vitesseLineaireMax)
                                {
                                    ghostLocationRefTerrain.Vx += accelLineaire / 50; // -!-   <_>     o-|-o     °_°     o_o 
                                }
                                else
                                {
                                    ghostLocationRefTerrain.Vx = ghostLocationRefTerrain.Vx;
                                }
                            }
                            else
                            {
                                ghostLocationRefTerrain.Vx -= accelLineaire / 50;
                            }
                        }
                    }
                    else
                    {
                        if (ghostLocationRefTerrain.Vx > 0)
                        {
                            ghostLocationRefTerrain.Vx += accelLineaire / 50;
                        }
                        else
                        {
                            if (Math.Abs(distEcart) > distArret)
                            {
                                if (ghostLocationRefTerrain.Vx > -vitesseLineaireMax)
                                {
                                    ghostLocationRefTerrain.Vx -= accelLineaire / 50; // -!-   <_>     o-|-o     °_°     o_o 
                                }
                                else
                                {
                                    ghostLocationRefTerrain.Vx = ghostLocationRefTerrain.Vx;
                                }
                            }
                            else
                            {
                                ghostLocationRefTerrain.Vx += accelLineaire / 50;
                            }
                        }
                    }
                    if (Math.Abs(distEcart) < tolerancedist)
                    {
                        state = ghostState.avance_recule;
                    }
                    break;
            }


            //pt projeté cible (r) = produit scalaire entre vecteur cible et vecteur ghost : theta = theta du ghost
            ptCibleprojete = (wayPointLocation.X * ghostLocationRefTerrain.X + wayPointLocation.Y * ghostLocationRefTerrain.Y) / Math.Sqrt(Math.Pow((ghostLocationRefTerrain.X), 2) + Math.Pow((ghostLocationRefTerrain.Y), 2));
            cibleProjetee.X = ptCibleprojete * Math.Cos(ghostLocationRefTerrain.Theta);
            cibleProjetee.Y = ptCibleprojete * Math.Sin(ghostLocationRefTerrain.Theta);
            cibleProjetee.Theta = ghostLocationRefTerrain.Theta;


            distRobotCibleProjeteeX = cibleProjetee.X - ghostLocationRefTerrain.X;
            distRobotCibleProjeteeY = cibleProjetee.Y - ghostLocationRefTerrain.Y;
            distRobotCibleProjetee = Math.Sqrt(Math.Pow(distRobotCibleProjeteeX, 2) + Math.Pow(distRobotCibleProjeteeY, 2));

            angleGhostWaypoint = ghostLocationRefTerrain.Theta - wayPointLocation.Theta;


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
