using Constants;
using System;
using System.Collections.Generic;
using Utilities;

namespace WorldMap
{
    public class LocalWorldMap
    {     

        public virtual int RobotId { get; set; }
        public virtual int TeamId { get; set; }
        public virtual Location robotLocation { get; set; }
        //public virtual RoboCupRobotRole robotRole { get; set; }
        //public virtual BallHandlingState ballHandlingState { get; set; }
        public virtual string messageDisplay { get; set; }

        public virtual PlayingSide playingSide { get; set; }
        public virtual Location robotGhostLocation { get; set; }
        public virtual Location destinationLocation { get; set; }
        public virtual Location waypointLocation { get; set; }
        //public virtual List<Location> ballLocationList { get; set; }
        public virtual List<LocationExtended> obstaclesLocationList { get; set; }
        public virtual List<PolarPointListExtended> lidarObjectList { get; set; }

        public virtual List<PointD> lidarMap { get; set; }
        public virtual List<PointD> lidarMapProcessed { get; set; }
        //public virtual Heatmap heatMapStrategy { get; set; }
        //public virtual Heatmap heatMapWaypoint { get; set; }

        public LocalWorldMap()
        {
            //Type = "LocalWorldMap";
        }

        public void Init()
        {
            robotLocation = new Location(0, 0, 0, 0, 0, 0);
            robotGhostLocation = new Location(0, 0, 0, 0, 0, 0); 
        }
    }

    public enum GameState
    {
        STOPPED,
        STOPPED_GAME_POSITIONING,
        PLAYING,
    }

    public enum StoppedGameAction
    {
        NONE,
        KICKOFF,
        KICKOFF_OPPONENT,
        FREEKICK,
        FREEKICK_OPPONENT,
        GOALKICK,
        GOALKICK_OPPONENT,
        THROWIN,
        THROWIN_OPPONENT,
        CORNER,
        CORNER_OPPONENT,
        PENALTY,
        PENALTY_OPPONENT,
        PARK,
        DROPBALL,
        GOTO,
        GOTO_OPPONENT,
    }
}
