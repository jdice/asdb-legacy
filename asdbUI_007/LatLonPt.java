import googlemapper.*;
import java.text.*;

public class LatLonPt{
	
	private double myLat;
	private double myLon;
	private double myRadius;
	private static GoogleMapper gMapper;
	private static NumberFormat decimalDegreeFormat = new DecimalFormat("###.00000");
	
	public static void setMapper(GoogleMapper theGMap){
		gMapper = theGMap;
	}
	
	public LatLonPt(){
		myLat = 0.0d;
		myLon = 0.0d;
		myRadius = 0.0d;
	}
	
	public LatLonPt(double lat, double lon){
		myLat = lat;
		myLon = lon;
		myRadius = 0.0d;
	}
	
	public LatLonPt(double lat, double lon, double radius){
		myLat = lat;
		myLon = lon;
		myRadius = radius;
	}
	
	public LatLonPt(XYPt xy){
		myLat = gMapper.y2lat(xy.getY());
		myLon = gMapper.x2lon(xy.getX());
		myRadius = xy.getRadius();
	}
	
	public double getLat(){
		return myLat;
	}
	
	public double getLon(){
		return myLon;
	}
	
	public double getRadius(){
		return myRadius;
	}
	
	public LatLonPt lerp(LatLonPt otherPt, float amount){
		double resultLat = this.getLat()*amount + otherPt.getLat()*(1-amount);
		double resultLon = this.getLon()*amount + otherPt.getLon()*(1-amount);
		double resultRadius = this.getRadius();
		LatLonPt resultPt = new LatLonPt(resultLat, resultLon, resultRadius);
		return resultPt;
	}
	
	public double distanceTo(LatLonPt otherPt){
		double deltaLat = otherPt.getLat() - this.getLat();
		double deltaLon = otherPt.getLon() - this.getLon();
		double distance = Math.sqrt(Math.pow(deltaLat,2)+Math.pow(deltaLon,2));
		return distance;
	}
	
	public double angleTo(LatLonPt otherPt){
		double deltaLat = otherPt.getLat() - this.getLat();
		double deltaLon = otherPt.getLon() - this.getLon();
		double angle = Math.atan2(deltaLat, deltaLon);
		return angle;
	}
	
	public String toString(){
		//String data = (new Double(this.getLat())).toString() + ", " + (new Double(this.getLon())).toString();
		String data = decimalDegreeFormat.format(this.getLat()) + ", " + decimalDegreeFormat.format(this.getLon());
		return data;
	}
	
}
