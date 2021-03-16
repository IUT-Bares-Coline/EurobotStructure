using EventArgsLibrary;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Utilities;

namespace Positioning2WheelsNS
{
    public class Positioning2Wheels
    {
        int robotId;
        Location posRobotRefTerrain = new Location(1, 1, Math.PI/ 4, 0.2, 0.5, 0.2);

        public Positioning2Wheels(int id)
        {
            robotId = id;
        }

        double rayonRoue = 0.027;
        //double ecartRoues =

        public void OnOdometryRobotSpeedReceived(object sender, PolarSpeedArgs e)
        {
            //Vtheta = (Vx-Vy)/2* ecartRoues;




            //https://lucidar.me/fr/mechanics/geometric-model-for-differential-wheeled-mobile-robot/
            //Vx { get; set; }
            //Vy { get; set; }
            //Vtheta { get; set; }
            //angleMotor1;
            //angleMotor2;

            //calcul de la nouvelle position


            //posRobotRefTerrain; = qqchose
            OnCalculatedLocation(robotId, posRobotRefTerrain);
        }

        //Output events
        public event EventHandler<LocationArgs> OnCalculatedLocationEvent;
        public virtual void OnCalculatedLocation(int id, Location locationRefTerrain)
        {
            var handler = OnCalculatedLocationEvent;
            if (handler != null)
            {
                handler(this, new LocationArgs { RobotId = id, Location = locationRefTerrain });
            }
        }
    }
}
