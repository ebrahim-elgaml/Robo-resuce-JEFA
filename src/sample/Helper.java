package sample;



import java.awt.Rectangle;
import java.util.ArrayList;




import rescuecore2.misc.Pair;
import rescuecore2.misc.geometry.GeometryTools2D;
import rescuecore2.misc.geometry.Point2D;
import rescuecore2.misc.geometry.Line2D;
import rescuecore2.misc.geometry.Vector2D;


public class Helper {
	// get the four vertices of a rectangle.
	public static ArrayList<Point2D> getRectangleVertices(Rectangle r){
		ArrayList<Point2D> result = new ArrayList<Point2D>();
		result.add(new Point2D(r.x,r.y));
		result.add(new Point2D(r.x+r.width,r.y));
		result.add(new Point2D(r.x+r.width,r.y + r.height));
		result.add(new Point2D(r.x,r.y + r.height));
		return result;
	}
	/*
	 * p3---l2---p2
	 * |        |
	 * l3       l1
	 * |        |
	 * p0---l0---p1
	 * 
	 */
	// get the four edges of a rectangle.
	public static ArrayList<Line2D> getRectangleLines(Rectangle r){
		ArrayList<Point2D> points =getRectangleVertices(r);
		ArrayList<Line2D> lines = new ArrayList<Line2D>();
		lines.add(new Line2D(points.get(0), points.get(1)));
		lines.add(new Line2D(points.get(1), points.get(2)));
		lines.add(new Line2D(points.get(2), points.get(3)));
		lines.add(new Line2D(points.get(3), points.get(0)));
		return lines;
	}
	// get all parallel edges of rectangle to a certain line.
	public static ArrayList<Line2D> getParralelLines(Rectangle r , Line2D l){
		ArrayList<Line2D> result = new ArrayList<Line2D>();
		ArrayList<Line2D> rectangleLines = getRectangleLines(r);
		for(Line2D ll: rectangleLines){
			if(GeometryTools2D.parallel(ll, l)){
				result.add(ll);
			}
		}
		return result;
	}
	// get the minimum distance between two rectangles.
	public static double getMinDistanceBetweenRectangles(Rectangle r1, Rectangle r2){
		ArrayList<Line2D> lines1 = getRectangleLines(r1);
		ArrayList<Line2D> lines2 = getRectangleLines(r2);
		double maxDistance = -1;
		for(Line2D l1 : lines1){
			for(Line2D l2 : lines2)
				if(GeometryTools2D.parallel(l1, l2)){
					double d = GeometryTools2D.getDistance(l1.getEndPoint(), l2.getEndPoint());
					if(d > maxDistance){
						maxDistance = d;
					}
				}
		}
		if(maxDistance <= -1){
			return Double.MIN_VALUE;
		}else{
			return Math.abs(maxDistance);
		}
	}
	// get minimum distance between rectangle and group of parallel lines.
	public static double getMinDistanceRectanglesAndParallelLines(Rectangle r1, ArrayList<Line2D> lines2){
		ArrayList<Line2D> lines1 = getRectangleLines(r1);
		double maxDistance = -1;
		for(Line2D l1 : lines1){
			for(Line2D l2 : lines2)
				if(GeometryTools2D.parallel(l1, l2)){
					double d = GeometryTools2D.getDistance(l1.getEndPoint(), l2.getEndPoint());
					if(d > maxDistance){
						maxDistance = d;
					}
				}
		}
		if(maxDistance <= -1){
			return Double.MIN_VALUE;
		}else{
			return Math.abs(maxDistance);
		}
	}
	// get the minimum distance between two groups of parallel line.
	public static double getMinDistanceBetweenParallelLines(ArrayList<Line2D> lines1, ArrayList<Line2D> lines2){
		double maxDistance = -1;
		for(Line2D l1 : lines1){
			for(Line2D l2 : lines2)
				if(GeometryTools2D.parallel(l1, l2)){
					double d = GeometryTools2D.getDistance(l1.getEndPoint(), l2.getEndPoint());
					if(d > maxDistance){
						maxDistance = d;
					}
				}
		}
		if(maxDistance <= -1){
			return Double.MIN_VALUE;
		}else{
			return Math.abs(maxDistance);
		}
	}
	// get the type of angle between two vectors.
	public static AngleType angleToType(Vector2D v1, Vector2D v2){
		if(v1.dot(v2) < 0)
			return AngleType.OBTUSE;
		return AngleType.ACUTE;
//		if(GeometryTools2D.nearlyZero(Math.abs(90 -a))){
//			return AngleType.RIGHT;
//		}else if( a > 90 ){
//			return AngleType.OBTUSE;
//		}else{
//			return AngleType.ACUTE;
//		}
	}
	/*
	 * |\
	 * |Y\.
	 * |  /     .
	 * |X/
	 * |/
	 */
	// Get X and Y angles.
	public static Pair<AngleType, AngleType> pointWRTLine(Point2D p , Line2D l){
		Line2D l1 = new Line2D(l.getOrigin(), p);
		Line2D l2 = new Line2D(l.getEndPoint(), p);
		return new Pair<AngleType, AngleType>(angleToType(l.getDirection(), l1.getDirection()), angleToType(new Line2D(l.getEndPoint(), l.getOrigin()).getDirection(), l2.getDirection()));
	}
	// @return null if line is completely out 
	// @return the clipped line if it is within the range 
	public static Line2D lineIsWithinLine(Line2D l1, Line2D l2 ){
		Pair<AngleType, AngleType> pair1 = pointWRTLine(l2.getOrigin(), l1);
		Pair<AngleType, AngleType> pair2 = pointWRTLine(l2.getEndPoint(), l1);
		if((pair1.first() == pair1.second() && pair1.first() == AngleType.ACUTE) &&(pair2.first() == pair2.second() && pair2.first() == AngleType.ACUTE)){
			return l2;
		}
		if(pair1.first() == pair1.second() && pair1.first() == AngleType.ACUTE){
			if(pair2.first() != AngleType.ACUTE){
				return new Line2D(l2.getOrigin(), GeometryTools2D.getClosestPoint(l2, l1.getOrigin()));
			}else{
				return new Line2D(l2.getOrigin(), GeometryTools2D.getClosestPoint(l2, l1.getEndPoint()));
			}
		}
		if(pair2.first() == pair2.second() && pair2.first() == AngleType.ACUTE){
			if(pair1.first() != AngleType.ACUTE){
				return new Line2D(GeometryTools2D.getClosestPoint(l2, l1.getOrigin()),l2.getEndPoint());
			}else{
				return new Line2D(GeometryTools2D.getClosestPoint(l2, l1.getEndPoint()), l2.getEndPoint());
			}
		}
		if((pair1.first() == pair1.second() && pair1.first() != AngleType.ACUTE) &&(pair2.first() == pair2.second() && pair2.first() != AngleType.ACUTE)){
			return new Line2D(GeometryTools2D.getClosestPoint(l2, l1.getOrigin()), GeometryTools2D.getClosestPoint(l2, l1.getEndPoint()));
		}
		return null;
	}
	
//	public static Rectangle linesToRectangle(ArrayList<Line2D> lines){
//		ArrayList<Line2D> parallelLines = new ArrayList<Line2D>();
//		for(int  i = 0 ; i < lines.size(); i++){
//			for(int j = i + 1 ; j<lines.size(); j++ ){
//				if(GeometryTools2D.parallel(lines.get(i), lines.get(j))){
//					parallelLines.add(lines.get(i));
//					parallelLines.add(lines.get(j));
//					break;
//				}
//			}
//		}
//		if(parallelLines.size() == 2){
//			return new Rectangle(x, y, width, height);
//		}else{
//			return null;
//		}
//	}
	public static void main(String[]args){
		Line2D l1 = new Line2D(new Point2D(0,0), new Point2D(5,5));
		Pair<AngleType, AngleType> p = pointWRTLine(new Point2D(2,1), l1);
		System.out.println(p.first()+" , "+ p.second());
		Line2D l2 = new Line2D(new Point2D(2,1), new Point2D(2,4));
		System.out.println(lineIsWithinLine(l1, l2));
//		Rectangle bounds = new Rectangle(0, 0, 2, 2);
//		for(Point2D p : getRectangleVertices(bounds)){
//			System.out.println(p.getX() +" , "+p.getY());
//		}
//		System.out.println(bounds.contains(new Point(1,1)));
//		System.out.println(bounds.toString());
	}

	public enum AngleType {
		ACUTE, OBTUSE, RIGHT;
	}

}
