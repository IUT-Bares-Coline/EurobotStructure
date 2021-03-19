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

        //double rayonRoue = 0.027;
        //double ecartRoues = 0.244;
        double positionX = 0;
        double positionY = 0;
        double positionTheta = 0;


        public void OnOdometryRobotSpeedReceived(object sender, PolarSpeedArgs e)
        {
            
            //e.Vtheta = e.Vy * ecartRoues;

            positionTheta += e.Vtheta / 50;
            positionX += (e.Vx / 50) * Math.Cos(positionTheta);
            positionY += (e.Vx / 50) * Math.Sin(positionTheta);



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
