/* QA only. Calls actual pinned Trace/DrillItem/ConductionArea methods. */
import app.freerouting.board.facade.RoutingBoard;
import app.freerouting.board.model.items.*;
import app.freerouting.board.model.structure.*;
import app.freerouting.board.state.Communication;
import app.freerouting.board.trace.PolylineTrace;
import app.freerouting.core.library.Padstacks;
import app.freerouting.geometry.planar.*;
import app.freerouting.rules.*;
import java.util.Random;

public class NormalContactsOracle {
  record Spec(int kind, int net, int layer, int x, int y, int ex, int ey) {
    public String toString() { return kind+" "+net+" "+layer+" "+x+" "+y+" "+ex+" "+ey; }
  }
  static Item item(RoutingBoard b, Spec s) {
    IntPoint p=new IntPoint(s.x,s.y),q=new IntPoint(s.ex,s.ey);
    if(s.kind==0) {
      return b.insertTraceWithoutCleaning(new Polyline(new Point[]{p,q}),s.layer,20,
          new int[]{s.net},0,FixedState.UNFIXED);
    }
    if(s.kind==1) {
      var stack=new Padstacks(b.layerStructure).add(new IntBox(-20,-20,20,20),s.layer,s.layer);
      // insertItem, not insertVia: intentionally test unsplit trace midpoints.
      Via via=new Via(stack,p,new int[]{s.net},0,0,0,FixedState.UNFIXED,false,b);
      b.insertItem(via);return via;
    }
    return b.insertConductionArea(new PolylineArea(new IntBox(s.x,s.y,s.ex,s.ey),
        new PolylineShape[]{new IntBox(s.x+25,s.y+25,s.ex-25,s.ey-25)}),s.layer,
        new int[]{s.net},0,false,FixedState.UNFIXED);
  }
  static String point(Point p) {
    if(p==null)return "0";
    IntPoint q=(IntPoint)p;return "1 "+q.x+" "+q.y;
  }
  public static void main(String[] args) {
    Random r=new Random(230098);
    for(int i=0;i<2048;i++) {
      LayerStructure layers=new LayerStructure(new Layer[]{new Layer("top",true),new Layer("bottom",true)});
      var rules=new BoardRules(layers,ClearanceMatrix.getDefaultInstance(layers,0));
      var board=new RoutingBoard(new IntBox(-100000,-100000,100000,100000),layers,
          new PolylineShape[0],0,rules,new Communication());
      int x=(r.nextInt(9)-4)*25,y=(r.nextInt(9)-4)*25;
      int kind=i%3,otherKind=(i/3)%3,layer=(i/9)%2,otherLayer=i%7==0?1-layer:layer;
      Spec a=new Spec(kind,1,layer,x,y,x+100,y+(i%2==0?0:100));
      if(kind==2)a=new Spec(kind,1,layer,x,y,x+100,y+100);
      int dx=(i/18)%5*25,dy=(i/90)%5*25;
      Spec b=new Spec(otherKind,i%11==0?2:1,otherLayer,x+dx,y+dy,x+dx+100,y+dy+100);
      if(i%13==0 && kind==0 && otherKind==0)
        b=new Spec(0,b.net,otherLayer,a.ex,a.ey,a.x,a.y); // two common endpoints
      Item first=item(board,a),second=item(board,b);
      if(first==null || second==null)throw new IllegalStateException("degenerate case "+i);
      System.out.println("CONTACT "+a+" "+b+" "+first.getNormalContacts().contains(second)
          +" "+second.getNormalContacts().contains(first)+" "+point(first.normalContactPoint(second))
          +" "+point(second.normalContactPoint(first)));
    }
  }
}
