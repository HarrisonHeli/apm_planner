
#include <QtGlobal>

#include "WaypointNavigation.h"

/*static*/ QPointF
WaypointNavigation::p(float t,
                      const QPointF& p0,
                      const QPointF& m0,
                      const QPointF& p1,
                      const QPointF& m1)
{
    Q_ASSERT(0.0f <= t && t <= 1.0f);
    const float t2 = t * t;
    const float t3 = t2 * t;
    return (2*t3 - 3*t2 + 1) * p0
         + (t3 - 2*t2 + t) * m0
         + (-2*t3 + 3*t2) * p1
         + (t3 - t2) * m1;
}

/*static*/ QPointF
WaypointNavigation::toQPointF(const Waypoint& wp,
                              mapcontrol::MapGraphicItem& map)
{
    internals::PointLatLng pt(wp.getLatitude(), wp.getLongitude());
    core::Point local = map.FromLatLngToLocal(pt);
    return QPointF(local.X(), local.Y());
}


/*static*/
QPainterPath WaypointNavigation::path(QList<Waypoint*>& waypoints, mapcontrol::MapGraphicItem& map)
{
    /* Note: a PATH is a series of graphic lines drawn between a list of given pixel points.
     * Its is just a graphic and is not used for navigation.
     */
    QPainterPath aGraphicPath;

    Q_ASSERT(waypoints.size() > 0);
    if (waypoints.size() <= 1) return aGraphicPath; // cant draw anything with only one point

    /* WayPointsActionFilter is a list of waypoint action types that acts as a search filter.
     * If a waypoint is not on this list we wont draw a path to it.
     * We are looking at the "action" property of the waypoint object.
     */

    QList<int> WayPointsActionFilter; // This QList of waypoint actions to filter for.
    WayPointsActionFilter << MAV_CMD_NAV_WAYPOINT // Here corresponding Waypoints actions can be added.
                      << MAV_CMD_NAV_LOITER_UNLIM
                      << MAV_CMD_NAV_LOITER_TURNS
                      << MAV_CMD_NAV_LOITER_TO_ALT
                      << MAV_CMD_NAV_LOITER_TIME
                      << MAV_CMD_NAV_LAND;
                      //<< MAV_CMD_NAV_TAKEOFF;

    /* The first waypoint is normaly the HOME waypoint. its default value is Zero Lat, Zero Long
     * The Home waypoint is changed to the location that the UAV is at when the UAV is ARMED.
     *
     * When doing flight planing we dont have a ARMED UAV so the HOME waypoint is always set to the
     * equator off the coast of Africa. It does not make sence to draw the path from the HOME location.
     *
     * We should use the UASManager to test if the UAV is armed but it is getting messy and dificult to pass that
     * information to here.
     * So if, the LAT and LONG are both zero deg, we can assume the UAV is not armed and can drop the
     * HOME waypoint from the graphic path.
     *
     * If the UAV is connected and ARMED then the loaction is valid and should be shown.
     */

    int IndexToNextWaypoint = 0; // Used to link this for loop to the next for loop

    for ( int i = 0; i < waypoints.size(); ++ i)
        {
        const Waypoint& aWaypoint = *waypoints[i];
        if (WayPointsActionFilter.contains(aWaypoint.getAction())) // Apply the waypoint action filter
            {
            if (!((aWaypoint.getLatitude() == 0) && (aWaypoint.getLongitude() == 0)))
                {
                 QPointF aGraphicPoint = toQPointF (aWaypoint, map);
                 aGraphicPath.moveTo(aGraphicPoint);
                 IndexToNextWaypoint = i + 1;
                 break; //exit the for loop
                }
            }  //if Way
        } //for



    QPointF m1; // spline velocity at destination

    for (int i = IndexToNextWaypoint ; i < waypoints.size(); ++i)
    {
        const Waypoint& wp1 = *waypoints[i];
        QPointF p1 = toQPointF(wp1, map);   //QPointF is a pixle point on the map.

        if (WayPointsActionFilter.contains(wp1.getAction())) // Apply the waypoint action filter
        {
            aGraphicPath.lineTo(p1);
            continue;
        }

        if (wp1.getAction() != MAV_CMD_NAV_SPLINE_WAYPOINT)
        {
            continue;
        }

        const Waypoint& wp0 = *waypoints[i-1];
        QPointF p0 = toQPointF(wp0, map);
        const Waypoint& wp2 = i < waypoints.size() - 1 ? *waypoints[i+1] : *waypoints[i];
        QPointF p2 = toQPointF(wp2, map);

        // segment start types
        // stop - vehicle is not moving at origin
        // straight-fast - vehicle is moving, previous segment is straight.  vehicle will fly straight through the waypoint before beginning it's spline path to the next wp
        // spline-fast - vehicle is moving, previous segment is splined, vehicle will fly through waypoint but previous segment should have it flying in the correct direction (i.e. exactly parallel to position difference vector from previous segment's origin to this segment's destination)

        // calculate spline velocity at origin
        QPointF m0;
        if (i == 1 // home
            || ((wp0.getAction() != MAV_CMD_NAV_WAYPOINT && wp0.getAction() != MAV_CMD_NAV_SPLINE_WAYPOINT) || wp0.getParam1() != 0)) // loiter time
        {
            // if vehicle is stopped at the origin, set origin velocity to 0.1 * distance vector from origin to destination
            m0 = (p1 - p0) * 0.1f;
        }
        else
        {
            // look at previous segment to determine velocity at origin
            if (wp0.getAction() == MAV_CMD_NAV_WAYPOINT)
            {
                // previous segment is straight, vehicle is moving so vehicle should fly straight through the origin
                // before beginning it's spline path to the next waypoint.
                // Note: we are using the previous segment's origin and destination

                Q_ASSERT(i > 1);

                const Waypoint& wp_1 = *waypoints[i-2];
                QPointF p_1 = toQPointF(wp_1, map);

                m0 = (p0 - p_1);
            }
            else
            {
                // previous segment is splined, vehicle will fly through origin
                // we can use the previous segment's destination velocity as this segment's origin velocity
                // Note: previous segment will leave destination velocity parallel to position difference vector
                //       from previous segment's origin to this segment's destination)

                Q_ASSERT(wp1.getAction() == MAV_CMD_NAV_SPLINE_WAYPOINT);

                m0 = m1;
            }
        }

        // calculate spline velocity at destination (m1)
        if (i == waypoints.size() - 1
            || wp1.getParam1() != 0)
        {
            // if vehicle stops at the destination set destination velocity to 0.1 * distance vector from origin to destination
            m1 = (p1 - p0) * 0.1f;
        }
        else if (wp2.getAction() == MAV_CMD_NAV_WAYPOINT)
        {
            // if next segment is straight, vehicle's final velocity should face along the next segment's position
            m1 = (p2 - p1);
        }
        else if (wp2.getAction() == MAV_CMD_NAV_SPLINE_WAYPOINT)
        {
            // if next segment is splined, vehicle's final velocity should face parallel to the line from the origin to the next destination
            m1 = (p2 - p0);
        }
        else
        {
            // if vehicle stops at the destination set destination velocity to 0.1 * distance vector from origin to destination
            m1 = (p1 - p0) * 0.1f;
        }

        // code below ensures we don't get too much overshoot when the next segment is short
        float vel_len = length(m0 + m1);
        float pos_len = length(p1 - p0) * 4.0f;
        if (vel_len > pos_len)
        {
            // if total start+stop velocity is more than twice position difference
            // use a scaled down start and stop velocityscale the  start and stop velocities down
            float vel_scaling = pos_len / vel_len;
            m0 *= vel_scaling;
            m1 *= vel_scaling;
        }

        // draw spline
        for (float t = 0.0f; t <= 1.0f; t += 1/100.0f) // update_spline() called at 100Hz
        {
            aGraphicPath.lineTo(p(t, p0, m0, p1, m1));
        }
    }

    return aGraphicPath;
}
