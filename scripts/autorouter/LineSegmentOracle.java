/* QA only: exercise the pinned Freerouting LineSegment implementation directly. */
import app.freerouting.geometry.planar.*;
import java.util.*;

public class LineSegmentOracle {
  private static String line(Line value) {
    return ConvexGeometryOracle.line(value);
  }

  private static void lines(StringBuilder out, Line[] values) {
    out.append(values.length);
    for (Line value : values) out.append(' ').append(line(value));
  }

  private static void segment(StringBuilder out, LineSegment value) {
    out.append(line(value.getStartClosingLine())).append(' ')
       .append(line(value.getLine())).append(' ')
       .append(line(value.getEndClosingLine()));
  }

  private static void polyline(StringBuilder out, Polyline value) {
    lines(out, value.lines);
  }

  private static void simplex(StringBuilder out, TileShape value) {
    Simplex shape = value.toSimplex();
    out.append(shape.borderLineCount());
    for (int index = 0; index < shape.borderLineCount(); ++index) {
      out.append(' ').append(line(shape.borderLine(index)));
    }
  }

  private static void points(StringBuilder out, IntPoint[] values) {
    out.append(values.length);
    for (IntPoint value : values) out.append(' ').append(value.x).append(' ').append(value.y);
  }

  private static LineSegment fromPoints(int x1, int y1, int x2, int y2) {
    IntPoint start = new IntPoint(x1, y1);
    IntPoint end = new IntPoint(x2, y2);
    Line middle = new Line(start, end);
    Direction perpendicular = middle.direction().turn45Degree(2);
    return new LineSegment(new Line(start, perpendicular), middle,
                           new Line(end, perpendicular));
  }

  private static LineSegment rationalSegment(int kind, int dx, int dy) {
    return switch (kind % 4) {
      case 0 -> new LineSegment(
          new Line(dx - 35, dy - 50, dx - 28, dy + 53),
          new Line(dx - 40, dy - 17, dx + 43, dy + 24),
          new Line(dx + 34, dy - 51, dx + 50, dy + 49));
      case 1 -> new LineSegment(
          new Line(dx + 46, dy + 51, dx + 31, dy - 48),
          new Line(dx + 42, dy + 29, dx - 39, dy - 22),
          new Line(dx - 33, dy + 47, dx - 27, dy - 54));
      case 2 -> new LineSegment(
          new Line(dx - 54, dy - 31, dx + 49, dy - 22),
          new Line(dx - 19, dy - 43, dx + 22, dy + 45),
          new Line(dx - 49, dy + 32, dx + 53, dy + 39));
      default -> new LineSegment(
          new Line(dx - 48, dy + 37, dx + 55, dy + 31),
          new Line(dx - 36, dy + 28, dx + 47, dy - 35),
          new Line(dx - 51, dy - 39, dx + 52, dy - 48));
    };
  }

  private static LineSegment primary(int record, int dx, int dy) {
    int[][] endpoints = {
      {-42, -18, 47, 23}, {48, 25, -37, -21}, {-44, 0, 49, 0},
      {0, -43, 0, 46}, {-38, -38, 45, 45}, {42, -42, -41, 41},
      {-47, -13, 44, 29}, {-39, 37, 43, -26}, {-48, -31, 35, 46},
      {39, 44, -46, -32}, {-51, 9, 37, -41}, {-28, -49, 33, 52}
    };
    if (record % 5 == 4) return rationalSegment(record / 5, dx, dy);
    int[] p = endpoints[record % endpoints.length];
    return fromPoints(dx + p[0], dy + p[1], dx + p[2], dy + p[3]);
  }

  private static LineSegment other(int record, int dx, int dy, LineSegment primary) {
    return switch (record % 10) {
      case 0 -> fromPoints(dx - 4, dy - 58, dx + 4, dy + 58);
      case 1 -> fromPoints(dx - 59, dy + 3, dx + 61, dy - 4);
      case 2 -> primary;
      case 3 -> primary.opposite();
      case 4 -> fromPoints(dx - 65, dy - 55, dx - 49, dy - 38);
      case 5 -> fromPoints(dx - 52, dy + 48, dx + 55, dy - 51);
      case 6 -> fromPoints(dx - 58, dy - 5, dx + 58, dy + 21);
      case 7 -> fromPoints(dx - 13, dy - 64, dx + 29, dy + 62);
      case 8 -> fromPoints(dx + 57, dy - 51, dx + 69, dy + 49);
      default -> fromPoints(dx - 61, dy + 29, dx + 63, dy + 34);
    };
  }

  private static TileShape shape(int record, int dx, int dy) {
    return switch (record % 4) {
      case 0 -> new IntBox(dx - 21, dy - 16, dx + 23, dy + 18);
      case 1 -> TileShape.getInstance(new Point[]{
          new IntPoint(dx - 24, dy - 18), new IntPoint(dx + 25, dy - 12),
          new IntPoint(dx + 8, dy + 27)});
      case 2 -> TileShape.getInstance(new Point[]{
          new IntPoint(dx - 27, dy - 9), new IntPoint(dx - 6, dy - 25),
          new IntPoint(dx + 26, dy - 13), new IntPoint(dx + 23, dy + 18),
          new IntPoint(dx - 13, dy + 25)});
      default -> TileShape.getInstance(new Point[]{
          new IntPoint(dx - 29, dy - 15), new IntPoint(dx + 18, dy - 24),
          new IntPoint(dx + 30, dy + 6), new IntPoint(dx + 4, dy + 28),
          new IntPoint(dx - 25, dy + 17)});
    };
  }

  private static double canonicalZero(double value) {
    return value == 0 ? 0 : value;
  }

  public static void main(String[] args) throws Exception {
    Random random = new Random(230106);
    for (int record = 0; record < 384; ++record) {
      int dx = random.nextInt(101) - 50;
      int dy = random.nextInt(101) - 50;
      LineSegment value = primary(record, dx, dy);
      LineSegment other = other(record, dx, dy, value);
      TileShape shape = shape(record, dx, dy);
      double width = 4.0 + record % 9;
      boolean toTheRight = (record & 1) != 0;
      IntPoint probe = new IntPoint(dx + record % 17 - 8, dy + record % 19 - 9);
      double newLength = 6.5 + record % 47;

      StringBuilder out = new StringBuilder("SEGMENT ");
      segment(out, value);
      out.append(' ');
      segment(out, other);
      out.append(' ').append(String.format(Locale.ROOT, "%.9f", width))
         .append(' ').append(toTheRight ? 1 : 0)
         .append(' ').append(probe.x).append(' ').append(probe.y)
         .append(' ').append(String.format(Locale.ROOT, "%.9f", newLength));
      out.append(' ').append(shape.borderLineCount());
      for (int index = 0; index < shape.borderLineCount(); ++index) {
        out.append(' ').append(line(shape.borderLine(index)));
      }
      int shapeLineIndex = record % shape.borderLineCount();
      out.append(' ').append(shapeLineIndex);

      out.append(' ').append(ConvexGeometryOracle.point(value.startPoint()))
         .append(' ').append(ConvexGeometryOracle.point(value.endPoint()));
      FloatPoint startApprox = value.startPointApprox();
      FloatPoint endApprox = value.endPointApprox();
      out.append(' ').append(String.format(Locale.ROOT, "%.9f %.9f %.9f %.9f",
          canonicalZero(startApprox.x), canonicalZero(startApprox.y),
          canonicalZero(endApprox.x), canonicalZero(endApprox.y)));
      out.append(' ');
      segment(out, value.opposite());
      out.append(' ');
      polyline(out, value.toPolyline());
      out.append(' ');
      simplex(out, value.toSimplex());
      out.append(' ').append(value.contains(probe) ? 1 : 0);
      IntBox box = value.boundingBox();
      out.append(' ').append(box.ll.x).append(' ').append(box.ll.y)
         .append(' ').append(box.ur.x).append(' ').append(box.ur.y);
      IntOctagon octagon = value.boundingOctagon();
      out.append(' ').append(octagon.leftX).append(' ').append(octagon.bottomY)
         .append(' ').append(octagon.rightX).append(' ').append(octagon.topY)
         .append(' ').append(octagon.upperLeftDiagonalX)
         .append(' ').append(octagon.lowerRightDiagonalX)
         .append(' ').append(octagon.lowerLeftDiagonalX)
         .append(' ').append(octagon.upperRightDiagonalX);
      LineSegment changed = value.changeLengthApprox(newLength);
      out.append(' ');
      segment(out, changed);
      out.append(' ').append(ConvexGeometryOracle.point(changed.endPoint()));
      Line[] intersections = value.intersection(other);
      out.append(' ');
      lines(out, intersections);
      out.append(' ').append(value.intersects(other) ? 1 : 0)
         .append(' ').append(value.overlaps(other) ? 1 : 0);
      out.append(' ');
      points(out, value.stairApproximation(width, toTheRight));
      out.append(' ');
      points(out, value.stairApproximation45(width, toTheRight));
      int[] borders = value.borderIntersections(shape);
      out.append(' ').append(borders.length);
      for (int border : borders) out.append(' ').append(border);
      out.append(' ');
      segment(out, value.sortEndpointsInXY());
      out.append(' ');
      segment(out, new LineSegment(shape, shapeLineIndex));
      System.out.println(out);
    }
  }
}
