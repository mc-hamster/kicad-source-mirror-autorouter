/* QA only. Instantiates pinned BatchFanout and reads its actual sorted sets.
 * Both sides use identical IU coordinates and 1000 IU/um. Do NOT scale only
 * the output coordinates: floating centroid/tie ordering is not scale invariant.
 */
import app.freerouting.autoroute.pipeline.BatchFanout;
import app.freerouting.board.facade.RoutingBoard;
import app.freerouting.board.model.items.Pin;
import app.freerouting.board.model.structure.*;
import app.freerouting.board.state.*;
import app.freerouting.core.library.*;
import app.freerouting.core.library.Package;
import app.freerouting.geometry.planar.*;
import app.freerouting.rules.*;
import app.freerouting.settings.RouterSettings;
import java.lang.reflect.*;
import java.util.*;

public class FanoutOrderOracle {
  public static void main(String[] args) throws Exception {
    var ctor=BatchFanout.class.getDeclaredConstructors()[0];ctor.setAccessible(true);
    Field sorted=BatchFanout.class.getDeclaredField("sortedComponents");sorted.setAccessible(true);
    Random r=new Random(230099);
    String[] orders={"outer_first","inner_first","distanceToClosestOnNet","surroundingsDensity","unknown"};
    for(int iteration=0;iteration<128;iteration++) {
      LayerStructure layers=new LayerStructure(new Layer[]{new Layer("top",true),new Layer("bottom",true)});
      var rules=new BoardRules(layers,ClearanceMatrix.getDefaultInstance(layers,0));
      var defaults=new Communication();
      var communication=new Communication(Unit.UM,1000,defaults.specctraParserInfo,
          defaults.coordinateTransform,defaults.idGenerator,defaults.observers);
      var board=new RoutingBoard(new IntBox(-33000000,-33000000,33000000,33000000),layers,
          new PolylineShape[0],0,rules,communication);
      board.library.padstacks=new Padstacks(layers);
      board.library.packages=new Packages(board.library.padstacks);
      var smd=board.library.padstacks.add(new IntBox(-10,-10,10,10),0,0);
      var th=board.library.padstacks.add(new IntBox(-10,-10,10,10),0,1);
      List<Pin> pins=new ArrayList<>();StringBuilder description=new StringBuilder();
      int componentCount=1+r.nextInt(7);
      for(int c=0;c<componentCount;c++) {
        int n=1+r.nextInt(9);Package.Pin[] packagePins=new Package.Pin[n];
        int[] nets=new int[n];boolean[] isSmd=new boolean[n];
        for(int p=0;p<n;p++) {
          int x=(r.nextInt(13)-6)*5000000,y=(r.nextInt(13)-6)*5000000;
          isSmd[p]=r.nextInt(5)!=0;nets[p]=r.nextInt(4);
          packagePins[p]=new Package.Pin("P"+p,isSmd[p]?smd.id:th.id,new IntVector(x,y),0);
        }
        var pack=board.library.packages.add(packagePins);
        var component=board.components.add(new IntPoint(0,0),0,true,pack);
        for(int p=0;p<n;p++) {
          Pin pin=board.insertPin(component.id,p,nets[p]==0?new int[0]:new int[]{nets[p]},0,FixedState.UNFIXED);
          pins.add(pin);var point=(IntPoint)pin.getCenter();
          description.append(' ').append(component.id).append(' ').append(p).append(' ').append(nets[p])
              .append(' ').append(isSmd[p]?1:0).append(' ').append(point.x).append(' ').append(point.y);
        }
      }
      for(int order=0;order<orders.length;order++) {
        var settings=new RouterSettings();settings.fanout.pinSortingOrder=orders[order];
        var fanout=ctor.newInstance(board,settings,null);
        List<Integer> actual=new ArrayList<>();
        for(Object component:(Iterable<?>)sorted.get(fanout)) {
          Field smdPins=component.getClass().getDeclaredField("smdPins");smdPins.setAccessible(true);
          for(Object pin:(Iterable<?>)smdPins.get(component)) {
            Field boardPin=pin.getClass().getDeclaredField("boardPin");boardPin.setAccessible(true);
            actual.add(pins.indexOf(boardPin.get(pin)));
          }
        }
        StringBuilder line=new StringBuilder("FANOUT "+order+" "+pins.size()+description+" "+actual.size());
        for(int index:actual)line.append(' ').append(index);
        System.out.println(line);
      }
    }
  }
}
