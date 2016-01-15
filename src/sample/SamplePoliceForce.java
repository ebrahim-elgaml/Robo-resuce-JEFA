package sample;

import java.awt.Rectangle;
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

import rescuecore2.standard.entities.Edge;
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
    private final int minAllowedDistance = 7000;
    private final int maxAllowedDistance = 22000;

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
        ArrayList<Blockade> targets = getTargetBlockades();
        if (targets != null && !targets.isEmpty()) {
        	// For test clearing
        	System.out.println("SIZE = "+targets.size());
        	Blockade target = getNearestBlockadeToMe(targets);
        	int distance  = findDistanceTo(target , me().getX(), me().getY());
      	 //double distance = Math.sqrt(Math.pow(Math.abs(target.getX() - me().getX()),2) + Math.pow(Math.abs(target.getY() - me().getY()), 2));
        	System.out.println("DISTANCE " + distance);
        	if(distance < 6000){
	      		System.out.println("Clearing blockade " + target);
	        	Logger.info("Clearing blockade " + target);
	            sendSpeak(time, 1, ("Clearing " + target).getBytes());
	            List<Line2D> lines = GeometryTools2D.pointsToLines(GeometryTools2D.vertexArrayToPoints(target.getApexes()), true);
	            double best = Double.MAX_VALUE;
	            Point2D bestPoint = null;
	            Point2D origin = new Point2D(me().getX(), me().getY());
	            for (Line2D next : lines) {
	                Point2D closest = GeometryTools2D.getClosestPointOnSegment(next, origin);
	                double d = GeometryTools2D.getDistance(origin, closest);
	                if (d < best) {
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
        	System.out.println("GOOD PATH");
        }
        if(me().getX() == lastX && me().getY() == lastY){
        	lastX = me().getX();
            lastY = me().getY();
            sendMove(time, randomWalk());
            return;
        }
        // Plan a path to a blocked area
        List<EntityID> path = search.breadthFirstSearch(me().getPosition(), getBlockedRoads());
        if (path != null) {
        	Logger.info("Moving to target");
            Road r = (Road)model.getEntity(path.get(path.size() - 1));
//            Blockade b = getTargetBlockade(r, -1);
            ArrayList<Blockade> blockades = getTargetBlockades(r, -1);
            if(blockades != null && !blockades.isEmpty()){
            	Blockade b = blockades.get(blockades.size() - 1);
                lastX = me().getX();
                lastY = me().getY();
                sendMove(time, path, b.getX(), b.getY());
                Logger.debug("Path: " + path);
                Logger.debug("Target coordinates: " + b.getX() + ", " + b.getY());
                return;
            }
        }
        Logger.debug("Couldn't plan a path to a blocked road");
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
            	ArrayList<Blockade> targetBlockades = getTargetBlockades(r, distance); 
                if(targetBlockades!= null && !targetBlockades.isEmpty()){
                	result.add(r.getID());
                }
            	//result.add(r.getID());
            }
        }
        return result;
    }

    private ArrayList<Blockade> getTargetBlockades() {
        Logger.debug("Looking for target blockade");
        Area location = (Area)location();
        Logger.debug("Looking in current location");
        ArrayList<Blockade> result = getTargetBlockades(location, distance);
        if (result != null && !result.isEmpty()) {
            return result;
        }
        Logger.debug("Looking in neighbouring locations");
        for (EntityID next : location.getNeighbours()) {
            location = (Area)model.getEntity(next);
            result = getTargetBlockades(location, distance);
            if (result != null && !result.isEmpty()) {
                return result;
            }
        }
        return null;
    }
    private ArrayList<Blockade> getTargetBlockades(Area area, int maxDistance) {
        if (area == null || !area.isBlockadesDefined()) {
            return null;
        }
        List<EntityID> ids = area.getBlockades();
        int x = me().getX();
        int y = me().getY();
        for (EntityID next : ids) {
            Blockade b = (Blockade)model.getEntity(next);
            double d = findDistanceTo(b, x, y);
            if (maxDistance < 0 || d < maxDistance) {
                //                Logger.debug("In range");
            	ArrayList<Blockade> myBlockades = getAllCheckedBlockades(area, b);
            	if(isCriticalRangeLines(area, myBlockades)){
            		System.out.println("CRITICAL");
            		return myBlockades;
            	}
            }
        }
        return null;
    }

    // el block el msh b2dr a7ded el orintation bt3ha(momkn tkon intersection) btb2a critical.
    public ArrayList<Blockade> getAllCheckedBlockades(Area area, Blockade myBlockade){
    	ArrayList<Blockade> result = new ArrayList<Blockade>();
    	List<EntityID> ids = area.getBlockades();
    	ArrayList<Line2D> myLines = getParallelEdges(area, myBlockade);
    	if(myLines.isEmpty()){
    		result.add(myBlockade);
    		return result;
    	}
    	for (EntityID next : ids) {
            Blockade b = (Blockade)model.getEntity(next);
            if(myBlockade != b){
            	Rectangle bRec = b.getShape().getBounds();
            	ArrayList<Line2D> bLines = Helper.getRectangleLines(bRec);
            	for(Line2D myLin : myLines){
            		ArrayList<Line2D> bResultLines = new ArrayList<Line2D>();
            		for(Line2D bLin : bLines){
            			if(Helper.lineIsWithinLine(myLin, bLin) != null){
            				bResultLines.add(Helper.lineIsWithinLine(myLin, bLin));
            			}
                	}
            		if(bResultLines.size() >=2){
            			result.add(b);
            			break;
            		}
            	}
            }
        }
    	result.add(myBlockade);
		return result;
    }
    
    public boolean isCriticalRangeLines(Area area, ArrayList<Blockade> myBlockades){
    	// [refBlockade, ........, Area]
    	ArrayList<BlockOfLines> blockOfLines = new ArrayList<BlockOfLines>();
    	Blockade refBlockade = myBlockades.get(myBlockades.size() - 1);
    	Rectangle refRectangle = refBlockade.getShape().getBounds();
    	ArrayList<Line2D> refLines = Helper.getRectangleLines(refRectangle);
    	blockOfLines.add(new BlockOfLines(refLines));
    	for(int i = 0 ; i < myBlockades.size() - 1 ; i++ ){
    		ArrayList<Line2D> bLines = Helper.getRectangleLines(myBlockades.get(i).getShape().getBounds());
        	for(Line2D myLin : refLines){
        		ArrayList<Line2D> bResultLines = new ArrayList<Line2D>();
        		for(Line2D bLin : bLines){
        			if(Helper.lineIsWithinLine(myLin, bLin) != null){
        				bResultLines.add(Helper.lineIsWithinLine(myLin, bLin));
        			}
            	}
        		if(bResultLines.size() >=2){
        			blockOfLines.add(new BlockOfLines(bResultLines));
        			break;
        		}
        	}
    	}
    	blockOfLines.add(new BlockOfLines(Helper.getRectangleLines(area.getShape().getBounds())));
    	if(blockOfLines.size() == 2){
    		System.out.println("MIN DISTANCE CASE 2 = "+ Helper.getMinDistanceBetweenRectangles(refRectangle, area.getShape().getBounds()));
    		return isCriticalDistance(Helper.getMinDistanceBetweenRectangles(refRectangle, area.getShape().getBounds()));
//    		return Helper.getMinDistanceBetweenRectangles(refRectangle, area.getShape().getBounds()) <= minAllowedDistance;
    	}
    	double minDistance = Double.MIN_VALUE;
    	for(int i = 0 ; i < blockOfLines.size() - 1 ; i++){
    		for(int j = i +1 ; j < blockOfLines.size() ; j++){
    			double myDistance = Helper.getMinDistanceBetweenParallelLines(blockOfLines.get(i).lines, blockOfLines.get(j).lines);
    			if(myDistance >  minDistance){
    				minDistance = myDistance;
    			}
    		}
    	}
    	System.out.println("MIN DISTANCE = "+ minDistance);
//    	return isCriticalDistance(minDistance);
    	return isCriticalDistance(minDistance);
    }
    // NO Intersections
    public boolean validateRoute(Area a){
    	ArrayList<Edge> notPassableEdges = new ArrayList<Edge>();
    	for (Edge ee : a.getEdgesProperty().getValue()){
        	if(!ee.isPassable()){
        		notPassableEdges.add(ee);
        	}
        }
    	for(int i = 0 ; i < notPassableEdges.size() ; i++){
    		Line2D l1 = notPassableEdges.get(i).getLine();
    		for(int j = i + 1 ; j < notPassableEdges.size() ; j++){
    			Line2D l2 = notPassableEdges.get(j).getLine();
    			if(GeometryTools2D.positionOnLine(l1, l2.getOrigin()) == Double.NaN || GeometryTools2D.positionOnLine(l1, l2.getEndPoint()) == Double.NaN ){
    				if(GeometryTools2D.parallel(l1, l2)){
    					return true;
    				}
    			}
    		}
    	}
    	return false;
    }
    // To solve intersection roads
    public boolean isCriticalDistance(double myDistance){
    	if(myDistance <= minAllowedDistance){
    		return true;
    	}
    	if(myDistance >= maxAllowedDistance){
    		return true;
    	}
    	return false;
    }
    public ArrayList<Line2D> getParallelEdges(Area a, Blockade myBlockade){
    	ArrayList<Line2D> result = new ArrayList<Line2D>();
    	if(validateRoute(a)){
    		return result;
    	}
    	Rectangle myRectangle = myBlockade.getShape().getBounds();
    	for (Edge ee : a.getEdgesProperty().getValue()){
        	if(!ee.isPassable()){
        		ArrayList<Line2D> myLines = Helper.getParralelLines(myRectangle, ee.getLine());
        		for(Line2D ll : myLines){
        			if(!result.contains(ll)){
        				result.add(ll);
        			}
        		}
        	}
        }
    	return result;
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
    
	 public Blockade getNearestBlockadeToMe(ArrayList<Blockade> myBlockades) {
		 double x = me().getX();
		 double y = me().getY();
		 double bestDistance = 0;
		 Blockade best = null;
	     for (Blockade blockade : myBlockades) {
	    	 Pair<Integer, Integer> location = blockade.getLocation(model);
	         if (location == null) {
	        	 continue;
	         }
	         double dx = location.first() - x;
	         double dy = location.second() - y;
	         double distance = Math.hypot(dx, dy);
	         if (best == null || distance < bestDistance) {
	             bestDistance = distance;
	             best = blockade;
	         }
	     }
	    return best;
	}

    class BlockOfLines{
    	ArrayList<Line2D> lines;
    	BlockOfLines(ArrayList<Line2D> lines){
    		this.lines = lines;
    	}
    }
    // not used old methods.
//
//    /**
//       Get the blockade that is nearest this agent.
//       @return The EntityID of the nearest blockade, or null if there are no blockades in the agents current location.
//    */
//    
//    public EntityID getNearestBlockade() {
//        return getNearestBlockade((Area)location(), me().getX(), me().getY());
//    }
//    /**
//    Get the blockade that is nearest a point.
//    @param area The area to check.
//    @param x The X coordinate to look up.
//    @param y The X coordinate to look up.
//    @return The EntityID of the nearest blockade, or null if there are no blockades in this area.
//	 */
//	 
//	 public EntityID getNearestBlockade(Area area, int x, int y) {
//	     double bestDistance = 0;
//	     EntityID best = null;
//	     Logger.debug("Finding nearest blockade");
//	     if (area.isBlockadesDefined()) {
//	         for (EntityID blockadeID : area.getBlockades()) {
//	             Logger.debug("Checking " + blockadeID);
//	             StandardEntity entity = model.getEntity(blockadeID);
//	             
//	             Logger.debug("Found " + entity);
//	             if (entity == null) {
//	                 continue;
//	             }
//	             System.out.println("ID : "+entity.getID());
//	             Pair<Integer, Integer> location = entity.getLocation(model);
//	             Logger.debug("Location: " + location);
//	             if (location == null) {
//	                 continue;
//	             }
//	             double dx = location.first() - x;
//	             double dy = location.second() - y;
//	             double distance = Math.hypot(dx, dy);
//	             if (best == null || distance < bestDistance) {
//	                 bestDistance = distance;
//	                 best = entity.getID();
//	             }
//	         }
//	     }
//	     Logger.debug("Nearest blockade: " + best);
//	     return best;
//	 }
//	 
//     //Blockade target = getTargetBlockade();
////   Blockade target = (Blockade)model.getEntity(getNearestBlockade());
////   if (target != null) {
////   	 int distance  = findDistanceTo(target , me().getX(), me().getY());
////   	 //double distance = Math.sqrt(Math.pow(Math.abs(target.getX() - me().getX()),2) + Math.pow(Math.abs(target.getY() - me().getY()), 2));
////   	 //System.out.println("DISTANCE : "+distance);
////   	 if(distance < 2500){
////       	//System.out.println(me().getID()+":"+ target);
////       	Logger.info("Clearing blockade " + target);
////           sendSpeak(time, 1, ("Clearing " + target).getBytes());
////           //sendClear(time, target.getX(), target.getY());
////           List<Line2D> lines = GeometryTools2D.pointsToLines(GeometryTools2D.vertexArrayToPoints(target.getApexes()), true);
////           double best = Double.MAX_VALUE;
////           Point2D bestPoint = null;
////           Point2D origin = new Point2D(me().getX(), me().getY());
////           for (Line2D next : lines) {
////               Point2D closest = GeometryTools2D.getClosestPointOnSegment(next, origin);
////               double d = GeometryTools2D.getDistance(origin, closest);
////               if (d < best) {
////               //if (d < 100) {
////                   best = d;
////                   bestPoint = closest;
////               }
////           }
////	        Vector2D v = bestPoint.minus(new Point2D(me().getX(), me().getY()));
////	        v = v.normalised().scale(1000000);
////	        sendClear(time, (int)(me().getX() + v.getX()), (int)(me().getY() + v.getY()));
////	        return;
////   	}
////   }else{
////   	System.out.println("At time step "+time +" No blockade");
////   
////   }
//   //Blockade target = getTargetBlockade();
//	 
//
//	    private Blockade getTargetBlockade() {
//	        Logger.debug("Looking for target blockade");
//	        Area location = (Area)location();
//	        Logger.debug("Looking in current location");
//	        Blockade result = getTargetBlockade(location, distance);
//	        if (result != null) {
//	            return result;
//	        }
//	        Logger.debug("Looking in neighbouring locations");
//	        for (EntityID next : location.getNeighbours()) {
//	            location = (Area)model.getEntity(next);
//	            result = getTargetBlockade(location, distance);
//	            if (result != null) {
//	                return result;
//	            }
//	        }
//	        return null;
//	    }
//	    private Blockade getTargetBlockade(Area area, int maxDistance) {
//	        //        Logger.debug("Looking for nearest blockade in " + area);
//	        if (area == null || !area.isBlockadesDefined()) {
//	            //            Logger.debug("Blockades undefined");
//	            return null;
//	        }
//	        List<EntityID> ids = area.getBlockades();
//	        // Find the first blockade that is in range.
//	        int x = me().getX();
//	        int y = me().getY();
//	        for (EntityID next : ids) {
//	            Blockade b = (Blockade)model.getEntity(next);
//	            double d = findDistanceTo(b, x, y);
//	            //            Logger.debug("Distance to " + b + " = " + d);
//	            if (maxDistance < 0 || d < maxDistance) {
//	                //                Logger.debug("In range");
//	            	ArrayList<Blockade> myBlockades = getAllCheckedBlockades(area, b);
//	            	System.out.println("SIZE : "+myBlockades.size());
//	                return b;
//	            }
//	        }
//	        //        Logger.debug("No blockades in range");
//	        return null;
//	    }
	    
}

