/* QA only: calls actual pinned TraceShover.springOverObstacles on real boards. */
import app.freerouting.board.facade.RoutingBoard;
import app.freerouting.board.optimize.TraceShover;
import app.freerouting.board.model.items.*;
import app.freerouting.board.model.structure.*;
import app.freerouting.board.state.Communication;
import app.freerouting.geometry.planar.*;
import app.freerouting.rules.*;
import java.util.Random;

public class SpringOverOracle {
  public static void main(String[] args) throws Exception {
    Random r=new Random(230101);
    for(int iteration=0;iteration<384;iteration++) {
      var layers=new LayerStructure(new Layer[]{new Layer("top",true),new Layer("bottom",true)});
      var rules=new BoardRules(layers,ClearanceMatrix.getDefaultInstance(layers,0));
      rules.setTraceAngleRestriction(AngleRestriction.NINETY_DEGREE);
      var board=new RoutingBoard(new IntBox(-1000,-1000,1000,1000),layers,new PolylineShape[0],0,rules,new Communication());
      int n=iteration%5,halfWidth=1+iteration%4;
      StringBuilder description=new StringBuilder("SPRING "+halfWidth+" "+board.clearanceValue(0,0,0)+" "+n);
      for(int j=0;j<n;j++) {
        int x=-60+40*j,y=r.nextInt(31)-15,w=8+r.nextInt(20),h=8+r.nextInt(20);
        if(iteration%13==0){x=-10+j*2;y=-10+j*2;w=h=30-j*4;} // nested
        if(iteration%17==0){x=-10+j*10;y=-10+j*10;w=h=30;} // overlapping
        var area=board.insertConductionArea(new IntBox(x,y,x+w,y+h),0,new int[]{2},0,true,FixedState.SYSTEM_FIXED);
        area.setIsObstacle(true);
        description.append(' ').append(area.getId()).append(' ').append(x).append(' ').append(y).append(' ').append(x+w).append(' ').append(y+h);
      }
      Point[] points=switch(iteration%6) {
        case 0 -> new Point[]{new IntPoint(-100,0),new IntPoint(100,0)};
        case 1 -> new Point[]{new IntPoint(0,-100),new IntPoint(0,100)};
        case 2 -> new Point[]{new IntPoint(100,0),new IntPoint(-100,0)};
        case 3 -> new Point[]{new IntPoint(-100,-30),new IntPoint(-100,10),new IntPoint(100,10),new IntPoint(100,-30)};
        case 4 -> new Point[]{new IntPoint(-100,-10),new IntPoint(90,-10),new IntPoint(90,40),new IntPoint(-100,40)};
        default -> new Point[]{new IntPoint(-100,0),new IntPoint(0,0)}; // blocked endpoint
      };
      description.append(' ').append(points.length);
      for(Point p:points){var q=(IntPoint)p;description.append(' ').append(q.x).append(' ').append(q.y);}
      var result=new TraceShover(board).springOverObstacles(new Polyline(points),halfWidth,0,new int[]{1},0,null);
      System.out.println(description+" "+ConvexGeometryOracle.polyline(result));
    }
  }
}
