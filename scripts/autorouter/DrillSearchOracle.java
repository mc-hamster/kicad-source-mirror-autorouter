/* QA only. Executes pinned Freerouting methods; never builds in the reference
 * checkout. Package access is used solely to inspect the actual maze queue.
 */
package app.freerouting.autoroute.maze;
import app.freerouting.autoroute.drill.*;
import app.freerouting.autoroute.expansion.*;
import app.freerouting.board.facade.RoutingBoard;
import app.freerouting.board.model.structure.*;
import app.freerouting.board.state.Communication;
import app.freerouting.geometry.planar.*;
import app.freerouting.rules.*;
import app.freerouting.settings.RouterSettings;
import java.lang.reflect.*;
import java.util.*;

public class DrillSearchOracle {
  static String box(IntBox b) { return b.ll.x+" "+b.ll.y+" "+b.ur.x+" "+b.ur.y; }
  static IntBox randomBox(Random r) {
    int x=r.nextInt(2001)-1000,y=r.nextInt(2001)-1000;
    return new IntBox(x,y,x+r.nextInt(800),y+r.nextInt(800));
  }
  static RoutingBoard board(IntBox bounds) {
    LayerStructure layers=new LayerStructure(new Layer[]{new Layer("top",true),new Layer("bottom",true)});
    BoardRules rules=new BoardRules(layers,ClearanceMatrix.getDefaultInstance(layers,0));
    rules.setTraceAngleRestriction(AngleRestriction.NINETY_DEGREE);
    return new RoutingBoard(bounds,layers,new PolylineShape[0],0,rules,new Communication());
  }
  public static void main(String[] args) throws Exception {
    Random r=new Random(230091);
    for(int i=0;i<256;i++) {
      IntBox d=randomBox(r),hole=randomBox(r);
      if(i%4==0)hole=d;
      TileShape[] result=d.cutout(hole);
      StringBuilder line=new StringBuilder("CUT "+box(d)+" "+box(hole)+" "+result.length);
      for(var piece:result)line.append(' ').append(box(piece.boundingBox()));
      System.out.println(line);
    }
    IntBox bounds=new IntBox(-1000,-1000,1000,1000);
    for(int i=0;i<256;i++) {
      int n=r.nextInt(18);TileShape[] holes=new TileShape[n];
      StringBuilder line=new StringBuilder("AREA "+box(bounds)+" "+n);
      for(int j=0;j<n;j++){holes[j]=randomBox(r);line.append(' ').append(box(holes[j].boundingBox()));}
      TileShape[] pieces=new PolylineArea(bounds,holes).splitToConvex();
      line.append(' ').append(pieces.length);
      for(var piece:pieces)line.append(' ').append(box(piece.boundingBox()))
          .append(' ').append(piece.centreOfGravity().round().getId());
      System.out.println(line);
    }
    Field pagesField=DrillPageArray.class.getDeclaredField("pages");pagesField.setAccessible(true);
    for(int i=0;i<256;i++) {
      int width=70+r.nextInt(700);IntBox query=randomBox(r);
      if(i%4==0) query=new IntBox(-1000,-1000,-1000,1000);
      if(i%4==1) query=new IntBox(1100,1100,1200,1200);
      DrillPageArray array=new DrillPageArray(board(bounds),width);
      DrillPage[][] pages=(DrillPage[][])pagesField.get(array);
      StringBuilder line=new StringBuilder("PAGES "+box(bounds)+" "+width+" "+box(query)+" "+pages.length*pages[0].length);
      for(var row:pages)for(var page:row)line.append(' ').append(box(page.shape)).append(' ').append(page.getId());
      var overlaps=array.overlappingPages(query);line.append(' ').append(overlaps.size());
      for(var page:overlaps)line.append(' ').append(box(page.shape));
      System.out.println(line);
    }
    Constructor<AutorouteControl> constructor=AutorouteControl.class.getDeclaredConstructor(
        RoutingBoard.class,RouterSettings.class,AutorouteControl.ExpansionCostFactor[].class);
    constructor.setAccessible(true);
    for(int i=0;i<256;i++) {
      RoutingBoard board=board(new IntBox(-10000,-10000,10000,10000));
      var room=new CompleteFreeSpaceExpansionRoom(board.boundingBox,0,1);
      RouterSettings settings=new RouterSettings(board);
      double h=1+r.nextInt(5),v=1+r.nextInt(5),g=r.nextInt(1000),via=r.nextInt(1000);
      var factors=new AutorouteControl.ExpansionCostFactor[]{new AutorouteControl.ExpansionCostFactor(h,v),new AutorouteControl.ExpansionCostFactor(v,h)};
      AutorouteControl ctrl=constructor.newInstance(board,settings,factors);
      ctrl.minNormalViaCost=via;ctrl.minCheapViaCost=via*.8;
      MazeSearchEngine search=new MazeSearchEngine(new AutorouteEngine(board,0,false),ctrl);
      search.destinationDistance.join(new IntBox(2000,2000,3000,3000),0);
      FloatPoint from=new FloatPoint(r.nextInt(2000)-1000,r.nextInt(2000)-1000);
      IntBox shape=new IntBox(-200,-300,400,500);
      DrillPage page=new DrillPage(shape,board);
      var source=new MazeListElement(new ExpansionDoor(room,room),0,null,0,g,g,room,
          new FloatLine(from,from),false,MazeSearchElement.Adjustment.NONE,false);
      var expansion=new MazeExpansionEngine(search);
      expansion.expandToDrillPage(page,source);
      MazeListElement pageEntry=search.mazeExpansionList.first();search.mazeExpansionList.clear();
      double pageRemaining=search.destinationDistance.calculate(shape.nearestPoint(from),0);
      System.out.println("PAGE_COST "+box(shape)+" "+from.x+" "+from.y+" "+g+" "+via+" "+h+" "+v+" "+pageRemaining+" "+pageEntry.expansionValue+" "+pageEntry.sortingValue);
      var drill=new ExpansionDrill(shape,new IntPoint(100,100),0,1);
      var previous=i%2==0?pageEntry:source;
      expansion.expandToDrill(drill,previous,0);
      MazeListElement drillEntry=search.mazeExpansionList.first();
      double drillRemaining=search.destinationDistance.calculate(drillEntry.shapeEntry.a,0);
      System.out.println("DRILL_COST "+box(shape)+" "+from.x+" "+from.y+" "+previous.expansionValue+" "+via+" "+(i%2==0?1:0)+" "+h+" "+v+" "+drillRemaining+" "+drillEntry.expansionValue+" "+drillEntry.sortingValue+" "+drillEntry.shapeEntry.a.x+" "+drillEntry.shapeEntry.a.y+" "+drill.getId());
    }
  }
}
