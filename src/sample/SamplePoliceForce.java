package sample;

import java.util.List;
import java.util.ArrayList;
import java.util.Collection;
import java.util.EnumSet;

import rescuecore2.worldmodel.EntityID;
import rescuecore2.worldmodel.ChangeSet;
import rescuecore2.messages.Command;
import rescuecore2.log.Logger;
import rescuecore2.misc.Pair;
import rescuecore2.misc.geometry.GeometryTools2D;
import rescuecore2.misc.geometry.Point2D;
import rescuecore2.misc.geometry.Line2D;
import rescuecore2.misc.geometry.Vector2D;

import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.standard.entities.StandardEntityURN;
import rescuecore2.standard.entities.Road;
import rescuecore2.standard.entities.Blockade;
import rescuecore2.standard.entities.PoliceForce;
import rescuecore2.standard.entities.Area;

/**
   A sample police force agent.
 */
public class SamplePoliceForce extends AbstractSampleAgent<PoliceForce> {
    private static final String DISTANCE_KEY = "clear.repair.distance";

    private int distance;
    private int lastX = -1;
    private int lastY = -1;

    @Override
    public String toString() {
        return "Sample police force";
    }

    @Override
    protected void postConnect() {
        super.postConnect();
        model.indexClass(StandardEntityURN.ROAD);
        distance = config.getIntValue(DISTANCE_KEY);
    }
    @Override
    protected void think(int time, ChangeSet changed, Collection<Command> heard) {
        if (time == config.getIntValue(kernel.KernelConstants.IGNORE_AGENT_COMMANDS_KEY)) {
            // Subscribe to channel 1
            sendSubscribe(time, 1);
        }
        for (Command next : heard) {
            Logger.debug("Heard " + next);
        }
        // Am I near a blockade?
        //Blockade target = getTargetBlockade();
        Blockade target = (Blockade)model.getEntity(getNearestBlockade());
        if (target != null) {
        	 int distance  = findDistanceTo(target , me().getX(), me().getY());
        	 //double distance = Math.sqrt(Math.pow(Math.abs(target.getX() - me().getX()),2) + Math.pow(Math.abs(target.getY() - me().getY()), 2));
        	 System.out.println("DISTANCE : "+distance);
        	 if(distance < 2500){
	        	//System.out.println(me().getID()+":"+ target);
	        	Logger.info("Clearing blockade " + target);
	            sendSpeak(time, 1, ("Clearing " + target).getBytes());
	            //sendClear(time, target.getX(), target.getY());
	            List<Line2D> lines = GeometryTools2D.pointsToLines(GeometryTools2D.vertexArrayToPoints(target.getApexes()), true);
	            double best = Double.MAX_VALUE;
	            Point2D bestPoint = null;
	            Point2D origin = new Point2D(me().getX(), me().getY());
	            for (Line2D next : lines) {
	                Point2D closest = GeometryTools2D.getClosestPointOnSegment(next, origin);
	                double d = GeometryTools2D.getDistance(origin, closest);
	                if (d < best) {
	                //if (d < 100) {
	                    best = d;
	                    bestPoint = closest;
	                }
	            }
		        Vector2D v = bestPoint.minus(new Point2D(me().getX(), me().getY()));
		        v = v.normalised().scale(1000000);
		        sendClear(time, (int)(me().getX() + v.getX()), (int)(me().getY() + v.getY()));
		        return;
        	}
        }else{
        	System.out.println("At time step "+time +" No blockade");
        
        }
        if(me().getX() == lastX && me().getY() == lastY){
        	lastX = me().getX();
            lastY = me().getY();
            sendMove(time, randomWalk());
            return;
        }
        
//        if (target != null) {
//        	//System.out.println(me().getID()+":"+ target);
//        	Logger.info("Clearing blockade " + target);
//            sendSpeak(time, 1, ("Clearing " + target).getBytes());
////            sendClear(time, target.getX(), target.getY());
//            List<Line2D> lines = GeometryTools2D.pointsToLines(GeometryTools2D.vertexArrayToPoints(target.getApexes()), true);
//            double best = Double.MAX_VALUE;
//            Point2D bestPoint = null;
//            Point2D origin = new Point2D(me().getX(), me().getY());
//            for (Line2D next : lines) {
//                Point2D closest = GeometryTools2D.getClosestPointOnSegment(next, origin);
//                double d = GeometryTools2D.getDistance(origin, closest);
//                if (d < best) {
//                //if (d < 100) {
//                	System.out.println("distance : "+d+"- Best : "+ best);
//                    best = d;
//                    bestPoint = closest;
//                }
//            }
//	        Vector2D v = bestPoint.minus(new Point2D(me().getX(), me().getY()));
//	        v = v.normalised().scale(1000000);
//	        sendClear(time, (int)(me().getX() + v.getX()), (int)(me().getY() + v.getY()));
//	        return;
//        }
        // Plan a path to a blocked area
        // feha 7aga ghalat
        List<EntityID> path = search.breadthFirstSearch(me().getPosition(), getBlockedRoads());
        if (path != null) {
        																																Logger.info("Moving to target");
            Road r = (Road)model.getEntity(path.get(path.size() - 1));
            Blockade b = getTargetBlockade(r, -1);
            lastX = me().getX();
            lastY = me().getY();
            sendMove(time, path, b.getX(), b.getY());
            Logger.debug("Path: " + path);
            Logger.debug("Target coordinates: " + b.getX() + ", " + b.getY());
            System.out.println("Target coordinates: " + b.getX() + ", " + b.getY());
            System.out.println("X : "+me().getX()+" Y : "+me().getY());
            System.out.println("Path: " + path);
            return;
        }
        Logger.debug("Couldn't plan a path to a blocked road");
        System.out.println("Couldn't plan a path to a blocked road");
        Logger.info("Moving randomly");
        lastX = me().getX();
        lastY = me().getY();
        sendMove(time, randomWalk());
    }

    @Override
    protected EnumSet<StandardEntityURN> getRequestedEntityURNsEnum() {
        return EnumSet.of(StandardEntityURN.POLICE_FORCE);
    }

    private List<EntityID> getBlockedRoads() {
        Collection<StandardEntity> e = model.getEntitiesOfType(StandardEntityURN.ROAD);
        List<EntityID> result = new ArrayList<EntityID>();
        for (StandardEntity next : e) {
            Road r = (Road)next;
            if (r.isBlockadesDefined() && !r.getBlockades().isEmpty()) {
                result.add(r.getID());
            }
        }
        System.out.println("NO of roads : " + e.size());
        return result;
    }

    private Blockade getTargetBlockade() {
        Logger.debug("Looking for target blockade");
        Area location = (Area)location();
        Logger.debug("Looking in current location");
        Blockade result = getTargetBlockade(location, distance);
        if (result != null) {
            return result;
        }
        Logger.debug("Looking in neighbouring locations");
        for (EntityID next : location.getNeighbours()) {
            location = (Area)model.getEntity(next);
            result = getTargetBlockade(location, distance);
            if (result != null) {
                return result;
            }
        }
        return null;
    }

    private Blockade getTargetBlockade(Area area, int maxDistance) {
        //        Logger.debug("Looking for nearest blockade in " + area);
        if (area == null || !area.isBlockadesDefined()) {
            //            Logger.debug("Blockades undefined");
            return null;
        }
        List<EntityID> ids = area.getBlockades();
        // Find the first blockade that is in range.
        int x = me().getX();
        int y = me().getY();
        for (EntityID next : ids) {
            Blockade b = (Blockade)model.getEntity(next);
            double d = findDistanceTo(b, x, y);
            //            Logger.debug("Distance to " + b + " = " + d);
            if (maxDistance < 0 || d < maxDistance) {
                //                Logger.debug("In range");
                return b;
            }
        }
        //        Logger.debug("No blockades in range");
        return null;
    }

    private int findDistanceTo(Blockade b, int x, int y) {
        //        Logger.debug("Finding distance to " + b + " from " + x + ", " + y);
        List<Line2D> lines = GeometryTools2D.pointsToLines(GeometryTools2D.vertexArrayToPoints(b.getApexes()), true);
        double best = Double.MAX_VALUE;
        Point2D origin = new Point2D(x, y);
        for (Line2D next : lines) {
            Point2D closest = GeometryTools2D.getClosestPointOnSegment(next, origin);
            double d = GeometryTools2D.getDistance(origin, closest);
            //            Logger.debug("Next line: " + next + ", closest point: " + closest + ", distance: " + d);
            if (d < best) {
                best = d;
                //                Logger.debug("New best distance");
            }

        }
        return (int)best;
    }

    /**
       Get the blockade that is nearest this agent.
       @return The EntityID of the nearest blockade, or null if there are no blockades in the agents current location.
    */
    
    public EntityID getNearestBlockade() {
        return getNearestBlockade((Area)location(), me().getX(), me().getY());
    }
    

    /**
       Get the blockade that is nearest a point.
       @param area The area to check.
       @param x The X coordinate to look up.
       @param y The X coordinate to look up.
       @return The EntityID of the nearest blockade, or null if there are no blockades in this area.
    */
    
    public EntityID getNearestBlockade(Area area, int x, int y) {
        double bestDistance = 0;
        EntityID best = null;
        Logger.debug("Finding nearest blockade");
        if (area.isBlockadesDefined()) {
            for (EntityID blockadeID : area.getBlockades()) {
                Logger.debug("Checking " + blockadeID);
                StandardEntity entity = model.getEntity(blockadeID);
                
                Logger.debug("Found " + entity);
                if (entity == null) {
                    continue;
                }
                System.out.println("ID : "+entity.getID());
                Pair<Integer, Integer> location = entity.getLocation(model);
                Logger.debug("Location: " + location);
                if (location == null) {
                    continue;
                }
                double dx = location.first() - x;
                double dy = location.second() - y;
                double distance = Math.hypot(dx, dy);
                if (best == null || distance < bestDistance) {
                    bestDistance = distance;
                    best = entity.getID();
                }
            }
        }
        Logger.debug("Nearest blockade: " + best);
        return best;
    }
    
}
