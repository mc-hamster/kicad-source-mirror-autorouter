/* QA oracle only: invokes the pinned Freerouting implementation, not a rewrite
 * of the expected algorithms. Build/run outside the read-only reference checkout.
 * Usage: javac -cp <source-pin.jar> -d <classes> RoomSearchOracle.java
 *        java -cp <source-pin.jar>:<classes> RoomSearchOracle > <fixture>
 */
import app.freerouting.autoroute.expansion.*;
import app.freerouting.board.facade.RoutingBoard;
import app.freerouting.autoroute.maze.AutorouteEngine;
import app.freerouting.autoroute.maze.MazeListElement;
import app.freerouting.autoroute.maze.MazeSearchElement;
import app.freerouting.autoroute.path.FoundConnectionLocator;
import app.freerouting.board.model.structure.*;
import app.freerouting.board.state.Communication;
import app.freerouting.board.searchtree.*;
import app.freerouting.datastructures.ShapeTree;
import app.freerouting.geometry.planar.*;
import app.freerouting.rules.*;
import java.lang.reflect.*;
import java.util.*;

public class RoomSearchOracle {
  static class Entry implements SearchTreeObject {
    final int id, layer, net;
    final IntBox shape;
    final boolean obstacle;
    Entry(int id, IntBox shape, int layer, int net, boolean obstacle) {
      this.id = id; this.shape = shape; this.layer = layer; this.net = net; this.obstacle = obstacle;
    }
    public int getId() { return id; }
    public boolean isObstacle(int n) { return obstacle && (net == 0 || net != n); }
    public boolean isTraceObstacle(int n) { return isObstacle(n); }
    public int shapeLayer(int index) { return layer; }
    public int treeShapeCount(ShapeTree tree) { return 1; }
    public TileShape getTreeShape(ShapeTree tree, int index) { return shape; }
    public void setSearchTreeEntries(ShapeTree.Leaf[] entries, ShapeTree tree) {}
    public int compareTo(Object other) { return Integer.compare(id, ((SearchTreeObject)other).getId()); }
  }
  static String box(IntBox b) { return b.ll.x+" "+b.ll.y+" "+b.ur.x+" "+b.ur.y; }
  static IntBox randomBox(Random r) {
    int x = r.nextInt(181)-90, y = r.nextInt(181)-90;
    return new IntBox(x, y, x+r.nextInt(90), y+r.nextInt(90));
  }
  static RoutingBoard board() {
    LayerStructure layers = new LayerStructure(new Layer[]{new Layer("top",true),new Layer("bottom",true)});
    return new RoutingBoard(new IntBox(-100,-100,100,100), layers, new PolylineShape[0], 0,
      new BoardRules(layers, ClearanceMatrix.getDefaultInstance(layers,0)), new Communication());
  }
  public static void main(String[] args) throws Exception {
    Random r = new Random(230090);
    Method calculate = SortedOrthogonalRoomNeighbours.class.getDeclaredMethod("calculateNeighbours",
        ExpansionRoom.class, int.class, ShapeSearchTree.class, int.class);
    calculate.setAccessible(true);
    Field touches = SortedOrthogonalRoomNeighbours.class.getDeclaredField("edgeInteriorTouchesObstacle");
    touches.setAccessible(true);
    Method gaps = SortedOrthogonalRoomNeighbours.class.getDeclaredMethod("calculateNewIncompleteRooms", AutorouteEngine.class);
    gaps.setAccessible(true);
    Field incompletes = AutorouteEngine.class.getDeclaredField("incompleteExpansionRooms");
    incompletes.setAccessible(true);
    RoutingBoard board = board();
    for (int test=0; test<256; ++test) {
      ShapeSearchTree90Degree tree = new ShapeSearchTree90Degree(board,0);
      int count = 3+r.nextInt(25);
      List<SearchTreeObject> items = new ArrayList<>();
      StringBuilder output = new StringBuilder("TREE ").append(count).append('\n');
      for (int id=1; id<=count; ++id) {
        IntBox shape = id%7==0 ? new IntBox(-20,-20,20,20) : randomBox(r); // insertion ties
        int layer = r.nextInt(2), net = r.nextInt(3), kind=r.nextInt(4);
        SearchTreeObject item = kind==1 ? new CompleteFreeSpaceExpansionRoom(shape,layer,id)
            : new Entry(id,shape,layer,net,kind!=2);
        tree.insert(item); items.add(item);
        output.append(id).append(' ').append(box(shape)).append(' ').append(layer).append(' ')
            .append(net).append(' ').append(kind).append('\n');
      }
      int remove = 1+r.nextInt(count);
      SearchTreeObject removed = items.get(remove-1);
      tree.remove(Arrays.stream(tree.toArray()).filter(l -> l.object == removed).toArray(ShapeTree.Leaf[]::new));
      output.append("REMOVE ").append(remove).append('\n');
      // Reinsert a different object after removal; stale handles must not alias it.
      IntBox reinsert = randomBox(r);
      tree.insert(new Entry(count+1,reinsert,0,0,true));
      output.append("INSERT ").append(count+1).append(' ').append(box(reinsert)).append('\n');
      ShapeTree.Leaf[] leaves = tree.toArray();
      output.append("VISIT ").append(leaves.length);
      for (int i=leaves.length-1; i>=0; --i) output.append(' ').append(((SearchTreeObject)leaves[i].object).getId());
      IntBox room=new IntBox(-100,-100,100,100), contained=randomBox(r);
      if(test%3==0) contained=new IntBox(contained.ll.x,contained.ll.y,contained.ll.x,contained.ll.y);
      int layer=r.nextInt(2), net=1, ignore=r.nextInt(count+1);
      SearchTreeObject ignored=ignore==0 ? null : items.get(ignore-1);
      IntBox ignoreShape=test%2==0 ? new IntBox(-50,-50,50,50) : null;
      output.append("\nCOMPLETE ").append(box(room)).append(' ').append(box(contained)).append(' ')
          .append(layer).append(' ').append(net).append(' ').append(ignore).append(' ').append(ignoreShape==null?0:1);
      if(ignoreShape!=null) output.append(' ').append(box(ignoreShape));
      Collection<IncompleteFreeSpaceExpansionRoom> completed=tree.completeShape(
          new IncompleteFreeSpaceExpansionRoom(room,layer,contained),net,ignored,ignoreShape);
      output.append("\nRESULT ").append(completed.size());
      for(var out:completed) output.append(' ').append(box(out.getShape().boundingBox()))
          .append(' ').append(box(out.getContainedShape().boundingBox()));
      // Evaluate ordering on actual completed rooms, plus boundary/corner cases.
      var valid=tree.completeShape(new IncompleteFreeSpaceExpansionRoom(room,layer,contained),net,null,null);
      IntBox nb=valid.isEmpty()?new IntBox(1000,1000,2000,2000):valid.iterator().next().getShape().boundingBox();
      var sorted=(SortedOrthogonalRoomNeighbours)calculate.invoke(null,
          new IncompleteFreeSpaceExpansionRoom(nb,layer,nb),net,tree,1000);
      output.append("\nNEIGHBOURS ").append(box(nb)).append(' ').append(sorted.sortedNeighbours.size());
      for(Object n:sorted.sortedNeighbours) {
        Field item=n.getClass().getDeclaredField("searchTreeObject"), intersection=n.getClass().getDeclaredField("intersection"),
            first=n.getClass().getDeclaredField("firstTouchingSide"), last=n.getClass().getDeclaredField("lastTouchingSide");
        for(Field f:List.of(item,intersection,first,last)) f.setAccessible(true);
        output.append(' ').append(((SearchTreeObject)item.get(n)).getId()).append(' ').append(box((IntBox)intersection.get(n)))
            .append(' ').append(first.getInt(n)).append(' ').append(last.getInt(n));
      }
      boolean[] edge=(boolean[])touches.get(sorted); int missing=-1;
      for(int i=0;i<4;++i) if(!edge[i]) {missing=i; break;}
      output.append(' ').append(missing);
      AutorouteEngine engine = new AutorouteEngine(board,0,false);
      if(!sorted.sortedNeighbours.isEmpty()) gaps.invoke(sorted,engine);
      @SuppressWarnings("unchecked")
      List<IncompleteFreeSpaceExpansionRoom> pending=(List<IncompleteFreeSpaceExpansionRoom>)incompletes.get(engine);
      output.append("\nGAPS ").append(pending==null?0:pending.size());
      if(pending!=null) for(var gap:pending) output.append(' ').append(box(gap.getShape().boundingBox()))
          .append(' ').append(box(gap.getContainedShape().boundingBox()));
      System.out.println(output);
    }
    for(int test=0;test<256;++test) {
      IntBox first=new IntBox(0,0,1000,1000);
      IntBox second=switch(test%4) {
        case 0 -> new IntBox(1000, r.nextInt(800), 2000, 1000);
        case 1 -> new IntBox(r.nextInt(800), 1000, 1000, 2000);
        case 2 -> new IntBox(500+r.nextInt(500),500+r.nextInt(500),2000,2000);
        default -> new IntBox(100,100,900,900);
      };
      double offset=1+r.nextInt(100);
      var door=new ExpansionDoor(new CompleteFreeSpaceExpansionRoom(first,0,1),
          new CompleteFreeSpaceExpansionRoom(second,0,2));
      var sections=door.getSectionSegments(offset);
      StringBuilder line=new StringBuilder("DOOR ").append(box(first)).append(' ').append(box(second))
          .append(' ').append(offset).append(' ').append(sections.length);
      for(var s:sections) line.append(' ').append(s.a.x).append(' ').append(s.a.y).append(' ').append(s.b.x).append(' ').append(s.b.y);
      System.out.println(line);
    }
    Method corner = FoundConnectionLocator.class.getDeclaredMethod("calculateAdditionalCorner",
        FloatPoint.class,FloatPoint.class,boolean.class,AngleRestriction.class);
    corner.setAccessible(true);
    for(int test=0;test<256;++test) {
      FloatPoint from=new FloatPoint((r.nextInt(801)-400)/4.0,(r.nextInt(801)-400)/4.0);
      FloatPoint to=new FloatPoint((r.nextInt(801)-400)/4.0,(r.nextInt(801)-400)/4.0);
      boolean horizontal=r.nextBoolean(),orthogonal=r.nextBoolean();
      FloatPoint result=(FloatPoint)corner.invoke(null,from,to,horizontal,
          orthogonal?AngleRestriction.NINETY_DEGREE:AngleRestriction.FORTYFIVE_DEGREE);
      System.out.println("CORNER "+from.x+" "+from.y+" "+to.x+" "+to.y+" "+(horizontal?1:0)+" "+(orthogonal?1:0)+" "+result.x+" "+result.y);
      double h=1+r.nextInt(20)/4.0,v=1+r.nextInt(20)/4.0;
      System.out.println("DISTANCE "+from.x+" "+from.y+" "+to.x+" "+to.y+" "+h+" "+v+" "+from.weightedDistance(to,h,v));
    }
    for(int test=0;test<256;++test) {
      int[] ids={1+r.nextInt(20),1+r.nextInt(20),1+r.nextInt(20),1+r.nextInt(20)};
      var leftDoor=new ExpansionDoor(new CompleteFreeSpaceExpansionRoom(new IntBox(0,0,100,100),0,ids[0]),
          new CompleteFreeSpaceExpansionRoom(new IntBox(100,0,200,100),0,ids[1]));
      var rightDoor=new ExpansionDoor(new CompleteFreeSpaceExpansionRoom(new IntBox(0,0,100,100),0,ids[2]),
          new CompleteFreeSpaceExpansionRoom(new IntBox(100,0,200,100),0,ids[3]));
      int lf=r.nextInt(4),rf=test%2==0?lf:r.nextInt(4),lg=r.nextInt(4),rg=test%3==0?lg:r.nextInt(4);
      int ls=r.nextInt(3),rs=r.nextInt(3);
      if(test%7==0) { rightDoor=leftDoor; rf=lf; rg=lg; rs=ls; ids[2]=ids[0]; ids[3]=ids[1]; }
      var line=new FloatLine(new FloatPoint(0,0),new FloatPoint(0,0));
      var left=new MazeListElement(leftDoor,ls,null,0,lg,lf,null,line,false,MazeSearchElement.Adjustment.NONE,false);
      var right=new MazeListElement(rightDoor,rs,null,0,rg,rf,null,line,false,MazeSearchElement.Adjustment.NONE,false);
      System.out.println("QUEUE "+lf+" "+lg+" "+ids[0]+" "+ids[1]+" "+ls+" "+rf+" "+rg+" "+ids[2]+" "+ids[3]+" "+rs+" "+left.compareTo(right));
    }
  }
}
