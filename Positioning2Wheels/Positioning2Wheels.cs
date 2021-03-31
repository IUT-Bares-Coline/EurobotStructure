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
        double samplingPeriod = 1/50.0; //période d'échantillonage
        Location posRobotRefTerrain = new Location(1, 1, Math.PI/ 4, 0.2, 0.5, 0.2);

        public Positioning2Wheels(int id)
        {
            robotId = id;
        }

        //double rayonRoue = 0.027;
        //double ecartRoues = 0.244;

        public void OnOdometryRobotSpeedReceived(object sender, PolarSpeedArgs e)
        {
       
            posRobotRefTerrain.Theta += e.Vtheta * samplingPeriod;
            posRobotRefTerrain.Vx = e.Vx * Math.Cos(posRobotRefTerrain.Theta);
            posRobotRefTerrain.Vy = e.Vx * Math.Sin(posRobotRefTerrain.Theta);
            posRobotRefTerrain.X += posRobotRefTerrain.Vx * samplingPeriod;
            posRobotRefTerrain.Y += posRobotRefTerrain.Vy * samplingPeriod;


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
