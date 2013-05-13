import googlemapper.*;
import java.text.*;

public class XYPt{
	
	private double myX;
	private double myY;
	private double myRadius;
	private static GoogleMapper gMapper;
	private static NumberFormat xyPosFormat = new DecimalFormat("#######.00");
	
	public static void setMapper(GoogleMapper theGMap){
		gMapper = theGMap;
	}
	
	public XYPt(){
		myX = 0.0d;
		myY = 0.0d;
		myRadius = 0.0d;
	}
	
	public XYPt(double x, double y){
		myX = x;
		myY = y;
		myRadius = 0.0d;
	}
	
	public XYPt(double x, double y, double radius){
		myX = x;
		myY = y;
		myRadius = radius;
	}
	
	public XYPt(LatLonPt latlon){
		myX = gMapper.lon2x(latlon.getLon());
		myY = gMapper.lat2y(latlon.getLat());
		myRadius = latlon.getRadius();
	}
	
	public double getX(){
		return myX;
	}
	
	public double getY(){
		return myY;
	}
	
	public double getRadius(){
		return myRadius;
	}
	
	public XYPt lerp(XYPt otherPt, float amount){
		double resultX = this.getX()*amount + otherPt.getX()*(1-amount);
		double resultY = this.getY()*amount + otherPt.getY()*(1-amount);
		double resultRadius = this.getRadius();
		XYPt resultPt = new XYPt(resultX, resultY, resultRadius);
		return resultPt;
	}
	
	public double distanceTo(XYPt otherPt){
		double deltaX = otherPt.getX() - this.getX();
		double deltaY = otherPt.getY() - this.getY();
		double distance = Math.sqrt(Math.pow(deltaX,2)+Math.pow(deltaY,2));
		return distance;
	}
	
	public double angleTo(XYPt otherPt){
		double deltaX = otherPt.getX() - this.getX();
		double deltaY = otherPt.getY() - this.getY();
		double angle = Math.atan2(deltaY, deltaX);
		return angle;
	}
	
	public void add(XYPt addPt){
		myX += addPt.getX();
		myY += addPt.getY();
	}
	
	public void add(double addX, double addY){
		myX += addX;
		myY += addY;
	}
	
	public String toString(){
		//String data = (new Double(this.getX())).toString() + ", " + (new Double(this.getY())).toString();
		String data = xyPosFormat.format(this.getX()) + ", " + xyPosFormat.format(this.getY());
		return data;
	}
	
}
