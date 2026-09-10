/* QA only: exercise the pinned Freerouting Polyline implementation directly. */
import app.freerouting.geometry.planar.*;
import java.util.*;

public class PolylineGeometryOracle {
  private static String line(Line value) {
    IntPoint a = (IntPoint) value.a;
    IntPoint b = (IntPoint) value.b;
    return a.x + " " + a.y + " " + b.x + " " + b.y;
  }

  private static void lines(StringBuilder out, Polyline value) {
    out.append(value.lines.length);
    for (Line current : value.lines) out.append(' ').append(line(current));
  }

  private static void simplex(StringBuilder out, TileShape value) {
    Simplex shape = value.toSimplex();
    out.append(shape.borderLineCount());
    for (int index = 0; index < shape.borderLineCount(); ++index) {
      out.append(' ').append(line(shape.borderLine(index)));
    }
  }

  private static Polyline fromPoints(int[][] coordinates) {
    Point[] points = new Point[coordinates.length];
    for (int index = 0; index < coordinates.length; ++index) {
      points[index] = new IntPoint(coordinates[index][0], coordinates[index][1]);
    }
    return new Polyline(points);
  }

  private static Polyline path(int kind, int dx, int dy) {
    return switch (kind) {
      case 0 -> fromPoints(new int[][]{{dx - 30, dy - 10}, {dx - 5, dy - 10},
                                       {dx - 5, dy + 22}, {dx + 31, dy + 22}});
      case 1 -> fromPoints(new int[][]{{dx - 30, dy - 20}, {dx - 12, dy - 2},
                                       {dx + 7, dy - 21}, {dx + 31, dy + 3}});
      case 2 -> fromPoints(new int[][]{{dx - 33, dy - 18}, {dx - 11, dy - 7},
                                       {dx + 4, dy + 19}, {dx + 28, dy + 12},
                                       {dx + 36, dy - 17}});
      case 3 -> new Polyline(new Line[]{
          new Line(dx - 29, dy - 31, dx - 24, dy - 9),
          new Line(dx - 31, dy - 14, dx + 27, dy - 3),
          new Line(dx + 18, dy - 28, dx + 25, dy + 29),
          new Line(dx + 32, dy + 14, dx - 23, dy + 25),
          new Line(dx - 19, dy + 31, dx - 17, dy + 2)});
      case 4 -> fromPoints(new int[][]{{dx - 34, dy}, {dx - 4, dy},
                                       {dx - 3, dy + 1}, {dx - 2, dy},
                                       {dx + 32, dy}});
      case 5 -> fromPoints(new int[][]{{dx - 31, dy - 17}, {dx - 13, dy + 7},
                                       {dx + 2, dy - 14}, {dx + 13, dy + 9},
                                       {dx + 34, dy - 2}});
      case 6 -> new Polyline(new Line[]{
          new Line(dx - 20, dy, dx + 20, dy),
          new Line(dx, dy - 20, dx, dy + 20),
          new Line(dx - 20, dy - 20, dx + 20, dy + 20)});
      default -> fromPoints(new int[][]{{dx - 38, dy - 7}, {dx - 18, dy + 13},
                                         {dx - 3, dy - 2}, {dx + 12, dy + 13},
                                         {dx + 34, dy - 9}});
    };
  }

  private static void point(StringBuilder out, Point value) throws Exception {
    out.append(ConvexGeometryOracle.point(value));
  }

  private static double canonicalZero(double value) {
    return value == 0 ? 0 : value;
  }

  public static void main(String[] args) throws Exception {
    Random random = new Random(230105);
    int[] widths = {0, 1, 2, 5, 9, 13};
    for (int record = 0; record < 384; ++record) {
      int dx = random.nextInt(101) - 50;
      int dy = random.nextInt(101) - 50;
      Polyline value = path(record % 8, dx, dy);
      int width = widths[record % widths.length];
      int from = record % 4 == 0 ? 1 : 0;
      int to = record % 5 == 0 ? value.lines.length - 2 : value.lines.length - 1;
      double queryX = dx + (record % 13) * 5.25 - 31.5;
      double queryY = dy + (record % 11) * 4.75 - 23.75;
      int tx = record % 9 - 4;
      int ty = record % 7 - 3;
      int splitIndex = 1 + record % (value.lines.length - 2);
      Line endLine = (record & 1) == 0
          ? new Line(dx + record % 17 - 8, dy - 70, dx + record % 17 + 5, dy + 70)
          : new Line(dx - 70, dy + record % 19 - 9, dx + 70, dy + record % 19 + 4);
      Polyline other = fromPoints(new int[][]{{dx + 90, dy + 80}, {dx + 110, dy + 63}});
      Point last = value.lastCorner();
      if (last instanceof IntPoint integral && (record & 1) == 0) {
        other = new Polyline(integral, new IntPoint(dx + 79, dy - 61));
      }
      IntPoint[] probes = {
          new IntPoint(dx, dy), new IntPoint(dx - 30, dy - 10),
          new IntPoint(dx + 31, dy + 22), new IntPoint(dx + 70, dy + 70)};

      StringBuilder out = new StringBuilder("POLY ");
      lines(out, value);
      out.append(' ').append(width).append(' ').append(from).append(' ').append(to);
      out.append(' ').append(String.format(Locale.ROOT, "%.9f %.9f", queryX, queryY));
      out.append(' ').append(tx).append(' ').append(ty);
      out.append(' ').append(splitIndex).append(' ').append(line(endLine)).append(' ');
      lines(out, other);
      for (IntPoint probe : probes) out.append(' ').append(probe.x).append(' ').append(probe.y);

      out.append(' ');
      lines(out, value);
      out.append(' ').append(value.cornerCount());
      for (int index = 0; index < value.cornerCount(); ++index) {
        out.append(' ');
        point(out, value.corner(index));
      }
      out.append(' ').append(value.isPoint() ? 1 : 0)
         .append(' ').append(value.isOrthogonal() ? 1 : 0)
         .append(' ').append(value.isMultipleOf45Degree() ? 1 : 0);
      out.append(' ').append(String.format(Locale.ROOT, "%.9f %.9f",
          value.lengthApprox(), value.lengthApprox(1, value.cornerCount() - 1)));
      IntBox box = value.boundingBox();
      out.append(' ').append(box.ll.x).append(' ').append(box.ll.y)
         .append(' ').append(box.ur.x).append(' ').append(box.ur.y);
      IntOctagon octagon = value.boundingOctagon(0, value.cornerCount() - 1);
      out.append(' ').append(octagon.leftX).append(' ').append(octagon.bottomY)
         .append(' ').append(octagon.rightX).append(' ').append(octagon.topY)
         .append(' ').append(octagon.upperLeftDiagonalX)
         .append(' ').append(octagon.lowerRightDiagonalX)
         .append(' ').append(octagon.lowerLeftDiagonalX)
         .append(' ').append(octagon.upperRightDiagonalX);
      FloatPoint nearest = value.nearestPointApprox(new FloatPoint(queryX, queryY));
      out.append(' ').append(String.format(Locale.ROOT, "%.9f %.9f",
          canonicalZero(nearest.x), canonicalZero(nearest.y)));
      for (IntPoint probe : probes) out.append(' ').append(value.contains(probe) ? 1 : 0);
      out.append(' ');
      lines(out, value.reverse());
      out.append(' ');
      lines(out, value.translateBy(new IntVector(tx, ty)));
      out.append(' ');
      lines(out, value.combine(other));
      Polyline[] split = value.split(splitIndex, endLine);
      if (split == null) {
        out.append(" -1");
      } else {
        out.append(" 2 ");
        lines(out, split[0]);
        out.append(' ');
        lines(out, split[1]);
      }
      int skipIndex = Math.min(Math.max(splitIndex, 1), value.lines.length - 2);
      out.append(' ');
      lines(out, value.skipLines(skipIndex, skipIndex));
      TileShape[] offsets = value.offsetShapes(width, from, to);
      out.append(' ').append(offsets.length);
      for (TileShape shape : offsets) {
        out.append(' ');
        simplex(out, shape);
      }

      Line probeLine = value.lines[1];
      Line translated = probeLine.translate(width - 3.5);
      out.append(' ').append(line(translated));
      out.append(' ').append(String.format(Locale.ROOT, "%.9f %.9f %.9f",
          probeLine.signedDistance(new FloatPoint(queryX, queryY)),
          new FloatPoint(queryX, queryY).projectionApprox(probeLine).x,
          new FloatPoint(queryX, queryY).projectionApprox(probeLine).y));
      out.append(' ');
      point(out, probeLine.perpendicularProjection(probes[0]));
      System.out.println(out);
    }
  }
}
