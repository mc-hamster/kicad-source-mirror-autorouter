/* QA only: actual pinned geometry methods, no reimplemented expectations. */
import app.freerouting.geometry.planar.*;
import java.math.BigInteger;
import java.lang.reflect.Field;
import java.util.Random;

public class ConvexGeometryOracle {
  public static String point(Point p) throws Exception {
    if(p instanceof IntPoint i)return i.x+" "+i.y+" 1";
    Field x=p.getClass().getDeclaredField("x"),y=p.getClass().getDeclaredField("y"),z=p.getClass().getDeclaredField("z");
    x.setAccessible(true);y.setAccessible(true);z.setAccessible(true);
    BigInteger a=(BigInteger)x.get(p),b=(BigInteger)y.get(p),c=(BigInteger)z.get(p);
    if(c.signum()==0)return "INF";
    BigInteger gcd=a.abs().gcd(b.abs()).gcd(c);return a.divide(gcd)+" "+b.divide(gcd)+" "+c.divide(gcd);
  }
  public static String line(Line l) {var a=(IntPoint)l.a;var b=(IntPoint)l.b;return a.x+" "+a.y+" "+b.x+" "+b.y;}
  public static String polyline(Polyline p) throws Exception {
    StringBuilder s=new StringBuilder(""+(p==null?-1:p.isEmpty()?0:p.cornerCount()));
    if(p!=null && !p.isEmpty())for(int i=0;i<p.cornerCount();i++)s.append(' ').append(point(p.corner(i)));
    return s.toString();
  }
  public static void main(String[] args) throws Exception {
    Random random=new Random(230100);
    for(int i=0;i<1024;i++) {
      int x=random.nextInt(201)-100,y=random.nextInt(201)-100;
      Line a=new Line(x,y,x+1+random.nextInt(30),y+random.nextInt(31));
      Line b=new Line(x+random.nextInt(31),y+random.nextInt(31),x-1-random.nextInt(30),y+random.nextInt(31));
      System.out.println("LINE "+line(a)+" "+line(b)+" "+point(a.intersection(b)));
    }
    for(int i=0;i<1024;i++) {
      TileShape shape;
      int dx=i%11,dy=i%7;
      if(i%3==0)shape=new IntBox(-20+dx,-15+dy,20+dx,15+dy);
      else if(i%3==1)shape=TileShape.getInstance(new Line[]{new Line(-17+dx,-15+dy,20+dx,-11+dy),new Line(20+dx,-11+dy,13+dx,23+dy),new Line(13+dx,23+dy,-17+dx,-15+dy)});
      else shape=TileShape.getInstance(new Line[]{new Line(-13+dx,-17+dy,24+dx,-12+dy),new Line(20+dx,-23+dy,10+dx,25+dy),new Line(18+dx,20+dy,-24+dx,10+dy),new Line(-23+dx,19+dy,-10+dx,-22+dy)});
      Point a=new IntPoint(random.nextInt(81)-40,random.nextInt(81)-40),b=new IntPoint(random.nextInt(81)-40,random.nextInt(81)-40);
      if(a.equals(b))b=new IntPoint(50,50);
      Polyline path=new Polyline(a,b);
      StringBuilder s=new StringBuilder("CONVEX "+path.lines.length);
      for(Line l:path.lines)s.append(' ').append(line(l));
      s.append(' ').append(shape.borderLineCount());
      for(int k=0;k<shape.borderLineCount();k++)s.append(' ').append(line(shape.borderLine(k)));
      s.append(' ').append(polyline(path));
      int[][] entrances=shape.entrancePoints(path);s.append(' ').append(entrances.length);
      for(var e:entrances)s.append(' ').append(e[0]).append(' ').append(e[1]);
      var pieces=shape.cutout(path);s.append(' ').append(pieces.length);
      for(var p:pieces)s.append(' ').append(polyline(p));
      System.out.println(s);
    }
  }
}
