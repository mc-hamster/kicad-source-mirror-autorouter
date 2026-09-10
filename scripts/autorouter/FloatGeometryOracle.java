/* QA only: exercise pinned Freerouting FloatPoint/FloatLine directly. */
import app.freerouting.geometry.planar.*;
import java.util.*;

public class FloatGeometryOracle {
  private static double canonicalZero(double value) {
    return value == 0 ? 0 : value;
  }

  private static void point(StringBuilder out, FloatPoint value) {
    out.append(String.format(Locale.ROOT, "%.9f %.9f",
        canonicalZero(value.x), canonicalZero(value.y)));
  }

  private static void intPoint(StringBuilder out, IntPoint value) {
    out.append(value.x).append(' ').append(value.y);
  }

  private static void line(StringBuilder out, FloatLine value) {
    point(out, value.a);
    out.append(' ');
    point(out, value.b);
  }

  private static void optionalPoint(StringBuilder out, FloatPoint value) {
    if (value == null) out.append(-1);
    else {
      out.append("2 ");
      point(out, value);
    }
  }

  private static void optionalLine(StringBuilder out, FloatLine value) {
    if (value == null) out.append(-1);
    else {
      out.append("4 ");
      line(out, value);
    }
  }

  private static int side(Side value) {
    if (value == Side.ON_THE_LEFT) return 1;
    if (value == Side.ON_THE_RIGHT) return -1;
    return 0;
  }

  public static void main(String[] args) {
    Random random = new Random(230108);
    for (int record = 0; record < 512; ++record) {
      double dx = random.nextInt(101) - 50 + (record % 7) * 0.125;
      double dy = random.nextInt(101) - 50 - (record % 11) * 0.0625;
      FloatPoint p = new FloatPoint(dx - 31.25 + record % 5,
                                    dy - 17.75 + record % 3);
      FloatPoint q = new FloatPoint(dx + 28.5 - record % 4,
                                    dy + 23.125 - record % 6);
      FloatPoint r = new FloatPoint(dx - 13.375 + record % 9,
                                    dy + 35.625 - record % 8);
      FloatPoint s = new FloatPoint(dx + 41.75 - record % 10,
                                    dy - 29.25 + record % 7);
      FloatLine first = new FloatLine(p, q);
      FloatLine second = switch (record % 5) {
        case 0 -> new FloatLine(r, s);
        case 1 -> new FloatLine(new FloatPoint(p.x + 3.5, p.y + 2.25),
                                new FloatPoint(q.x + 3.5, q.y + 2.25));
        case 2 -> first.opposite();
        case 3 -> new FloatLine(new FloatPoint(dx - 50, dy + 12),
                                new FloatPoint(dx + 52, dy + 12));
        default -> new FloatLine(new FloatPoint(dx + 7, dy - 55),
                                 new FloatPoint(dx + 7, dy + 57));
      };
      double distance = (record % 17 - 8) * 1.125;
      double horizontalWeight = 0.5 + (record % 7) * 0.375;
      double verticalWeight = 0.75 + (record % 9) * 0.25;
      int factor = record % 13 - 6;
      double angle = (record % 11 - 5) * Math.PI / 19.0;
      int horizontalGrid = record % 6;
      int verticalGrid = (record + 3) % 7;
      IntVector directionVector = switch (record % 8) {
        case 0 -> new IntVector(1, 0);
        case 1 -> new IntVector(1, 1);
        case 2 -> new IntVector(0, 1);
        case 3 -> new IntVector(-1, 1);
        case 4 -> new IntVector(-1, 0);
        case 5 -> new IntVector(-1, -1);
        case 6 -> new IntVector(0, -1);
        default -> new IntVector(1, -1);
      };
      Direction direction = Direction.getInstance(directionVector);
      int sectionCount = record % 7;
      double tolerance = (record % 4) * 0.125;
      double tangentRadius = record % 9 == 0 ? 100 : 2.5 + record % 13;
      double newSize = 1.25 + record % 31;
      double newLength = 0.75 + record % 37;
      FloatPoint circle0 = new FloatPoint(dx - 17.25, dy - 11.5);
      FloatPoint circle1 = new FloatPoint(dx + 8.75, dy + 19.25);
      FloatPoint circle2 = new FloatPoint(dx + 28.5, dy - 7.75);

      StringBuilder out = new StringBuilder("FGEOM ");
      for (FloatPoint value : new FloatPoint[]{p, q, r, s}) {
        out.append(String.format(Locale.ROOT, "%.17g %.17g ", value.x, value.y));
      }
      out.append(String.format(Locale.ROOT, "%.17g %.17g %.17g %.17g ",
          second.a.x, second.a.y, second.b.x, second.b.y));
      out.append(String.format(Locale.ROOT, "%.17g %.17g %.17g ",
          distance, horizontalWeight, verticalWeight));
      out.append(factor).append(' ')
         .append(String.format(Locale.ROOT, "%.17g ", angle))
         .append(horizontalGrid).append(' ').append(verticalGrid).append(' ')
         .append(directionVector.x).append(' ').append(directionVector.y).append(' ')
         .append(sectionCount).append(' ')
         .append(String.format(Locale.ROOT, "%.17g %.17g %.17g %.17g ",
             tolerance, tangentRadius, newSize, newLength));
      for (FloatPoint value : new FloatPoint[]{circle0, circle1, circle2}) {
        out.append(String.format(Locale.ROOT, "%.17g %.17g ", value.x, value.y));
      }

      IntOctagon octagon = FloatPoint.boundingOctagon(new FloatPoint[]{p, q, r, s});
      out.append(octagon.leftX).append(' ').append(octagon.bottomY).append(' ')
         .append(octagon.rightX).append(' ').append(octagon.topY).append(' ')
         .append(octagon.upperLeftDiagonalX).append(' ')
         .append(octagon.lowerRightDiagonalX).append(' ')
         .append(octagon.lowerLeftDiagonalX).append(' ')
         .append(octagon.upperRightDiagonalX);
      out.append(' ').append(String.format(Locale.ROOT,
          "%.9f %.9f %.9f %.9f %.9f",
          canonicalZero(p.sizeSquare()), canonicalZero(p.size()),
          canonicalZero(p.distanceSquare(q)), canonicalZero(p.distance(q)),
          canonicalZero(p.weightedDistance(q, horizontalWeight, verticalWeight))));
      out.append(' '); intPoint(out, p.round());
      out.append(' '); intPoint(out, p.roundToTheRight(direction));
      out.append(' '); intPoint(out, p.roundToGrid(horizontalGrid, verticalGrid));
      out.append(' '); intPoint(out, p.roundToTheLeft(direction));
      out.append(' '); point(out, p.add(q));
      out.append(' '); point(out, p.subtract(q));
      out.append(' ').append(String.format(Locale.ROOT, "%.9f",
          canonicalZero(p.scalarProduct(q, r))));
      out.append(' '); point(out, p.changeSize(newSize));
      out.append(' '); point(out, p.changeLength(q, newLength));
      out.append(' '); point(out, p.middlePoint(q));
      out.append(' ').append(side(p.sideOf(q, r)));
      out.append(' '); point(out, p.rotate(angle, r));
      out.append(' '); point(out, p.turn90Degree(factor));
      out.append(' '); point(out, p.turn90Degree(factor, r));
      out.append(' ').append(p.isContainedInBox(q, r, tolerance) ? 1 : 0);
      IntBox box = p.boundingBox();
      out.append(' ').append(box.ll.x).append(' ').append(box.ll.y)
         .append(' ').append(box.ur.x).append(' ').append(box.ur.y);
      FloatPoint[] tangents = p.tangentialPoints(q, tangentRadius);
      out.append(' ').append(tangents.length);
      for (FloatPoint tangent : tangents) {
        out.append(' '); point(out, tangent);
      }
      out.append(' '); optionalPoint(out, p.leftTangentialPoint(q, tangentRadius));
      out.append(' '); optionalPoint(out, p.rightTangentialPoint(q, tangentRadius));
      out.append(' '); point(out, circle0.circleCenter(circle1, circle2));
      out.append(' ').append(p.insideCircle(circle0, circle1, circle2) ? 1 : 0);

      out.append(' '); line(out, first.opposite());
      out.append(' '); line(out, first.adjustDirection(second));
      out.append(' '); optionalPoint(out, first.intersection(second));
      out.append(' '); line(out, first.translate(distance));
      out.append(' ').append(String.format(Locale.ROOT, "%.9f",
          canonicalZero(first.signedDistance(r))));
      out.append(' '); point(out, first.perpendicularProjection(r));
      out.append(' ').append(String.format(Locale.ROOT, "%.9f",
          canonicalZero(first.segmentDistance(r))));
      out.append(' '); optionalLine(out, first.segmentProjection(second));
      out.append(' '); optionalLine(out, first.segmentProjection2(second));
      out.append(' '); line(out, first.shrinkSegment(Math.abs(distance)));
      out.append(' '); point(out, first.nearestSegmentPoint(r));
      FloatLine[] sections = first.divideSegmentIntoSections(sectionCount);
      out.append(' ').append(sections.length);
      for (FloatLine section : sections) {
        out.append(' '); line(out, section);
      }
      System.out.println(out);
    }
  }
}
