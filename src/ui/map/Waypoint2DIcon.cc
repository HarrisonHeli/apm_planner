#include "Waypoint2DIcon.h"
#include "QsLog.h"
#include "opmapcontrol.h"
#include "QGC.h"
#include <QPainter>

Waypoint2DIcon::Waypoint2DIcon(mapcontrol::MapGraphicItem* map, mapcontrol::OPMapWidget* parent, qreal latitude, qreal longitude, qreal altitude, int listindex, QString name, QString description, int radius)
    : mapcontrol::WayPointItem(internals::PointLatLng(latitude, longitude), altitude, description, map),
    parent(parent),
    waypoint(NULL),
    radius(radius),
    showAcceptanceRadius(true),
    showOrbit(false),
    color(Qt::red)
{
    Q_UNUSED(name);

    SetHeading(0);
    SetNumber(listindex);
    this->setFlag(QGraphicsItem::ItemIgnoresTransformations,true);
    picture = QPixmap(radius+1, radius+1);
    autoreachedEnabled = false; // In contrast to the use in OpenPilot, we don't
                                // want to let the map interfere with the actual mission logic
                                // wether a WP is reached depends solely on the UAV's state machine
    drawIcon();
}

Waypoint2DIcon::Waypoint2DIcon(mapcontrol::MapGraphicItem* map, mapcontrol::OPMapWidget* parent, Waypoint* wp, const QColor& color, int listindex, int radius)
    : mapcontrol::WayPointItem(internals::PointLatLng(wp->getLatitude(), wp->getLongitude()), wp->getAltitude(), wp->getDescription(), map),
    parent(parent),
    waypoint(wp),
    radius(radius),
    showAcceptanceRadius(true),
    showOrbit(false),
    color(color)
{
    SetHeading(wp->getYaw());
    SetNumber(listindex);
    this->setFlag(QGraphicsItem::ItemIgnoresTransformations,true);
    picture = QPixmap(radius+1, radius+1);
    autoreachedEnabled = false; // In contrast to the use in OpenPilot, we don't
                                // want to let the map interfere with the actual mission logic
                                // wether a WP is reached depends solely on the UAV's state machine
    updateWaypoint();
}

Waypoint2DIcon::~Waypoint2DIcon()
{
}

void Waypoint2DIcon::SetHeading(float heading)
{
    mapcontrol::WayPointItem::SetHeading(heading);
    drawIcon();
}

void Waypoint2DIcon::updateWaypoint()
{
    if (waypoint) {
        // Store old size
        static QRectF oldSize;

        SetHeading(waypoint->getYaw());
        SetCoord(internals::PointLatLng(waypoint->getLatitude(), waypoint->getLongitude()));

        // QLOG_DEBUG() << "UPDATING WP:" << waypoint->getId() << "LAT:" << waypoint->getLatitude() << "LON:" << waypoint->getLongitude();

        SetDescription(waypoint->getDescription());
        SetAltitude(waypoint->getAltitude());
        // FIXME Add SetNumber (currently needs a separate call)
        drawIcon();
        QRectF newSize = boundingRect();

        // QLOG_DEBUG() << "WIDTH" << newSize.width() << "<" << oldSize.width();

        // If new size is smaller than old size, update surrounding
        if ((newSize.width() <= oldSize.width()) || (newSize.height() <= oldSize.height()))
        {
            // If the symbol size was reduced, enforce an update of the environment
//            update(oldSize);
            int oldWidth = oldSize.width() + 20;
            int oldHeight = oldSize.height() + 20;
            map->update(this->x()-10, this->y()-10, oldWidth, oldHeight);
            //// QLOG_DEBUG() << "UPDATING DUE TO SMALLER SIZE";
            //// QLOG_DEBUG() << "X:" << this->x()-1 << "Y:" << this->y()-1 << "WIDTH:" << oldWidth << "HEIGHT:" << oldHeight;
        }
        else
        {
            // Symbol size stayed constant or increased, use new size for update
            this->update();
        }
        oldSize = boundingRect();
    }
}

QRectF Waypoint2DIcon::boundingRect() const
{
    int loiter = 0;
    int acceptance = 0;
    internals::PointLatLng coord = (internals::PointLatLng)Coord();
    if (waypoint && showAcceptanceRadius &&
            ( (waypoint->getAction() == (int)MAV_CMD_NAV_WAYPOINT)
             ||(waypoint->getAction() == (int)MAV_CMD_NAV_SPLINE_WAYPOINT)) )
    {
        acceptance = map->metersToPixels(waypoint->getAcceptanceRadius(), coord);
    }
    if ((waypoint->getAction() == (int)MAV_CMD_NAV_LOITER_UNLIM) || (waypoint->getAction() == (int)MAV_CMD_NAV_LOITER_TIME) || (waypoint->getAction() == (int)MAV_CMD_NAV_LOITER_TURNS))
    {
        loiter = map->metersToPixels(waypoint->getLoiterOrbit(), coord);
    }

    if (waypoint && waypoint->getAction() == (int)MAV_CMD_NAV_LOITER_TO_ALT)
    {
        loiter = map->metersToPixels(waypoint->getParam2(), coord);
    }

    int width = qMax(picture.width()/2, qMax(loiter, acceptance));
    int height = qMax(picture.height()/2, qMax(loiter, acceptance));

    return QRectF(-width,-height,2*width,2*height);
}

void Waypoint2DIcon::drawIcon()
{
    picture.fill(Qt::transparent);
    QPainter painter(&picture);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::HighQualityAntialiasing, true);

    QFont font("Bitstream Vera Sans");
    int fontSize = picture.height()*0.8f;
    font.setPixelSize(fontSize);

    QFontMetrics metrics = QFontMetrics(font);
    int border = qMax(4, metrics.leading());
    painter.setFont(font);
    painter.setRenderHint(QPainter::TextAntialiasing);



    QPen pen1(Qt::black);
    pen1.setWidth(4);
    QPen pen2(color);
    pen2.setWidth(2);
    painter.setBrush(Qt::NoBrush);

    int penWidth = pen1.width();

    // DRAW WAYPOINT
    QPointF p(picture.width()/2, picture.height()/2);

    QPolygonF poly(4);
    // Top point
    poly.replace(0, QPointF(p.x(), p.y()-picture.height()/2.0f+penWidth/2));
    // Right point
    poly.replace(1, QPointF(p.x()+picture.width()/2.0f-penWidth/2, p.y()));
    // Bottom point
    poly.replace(2, QPointF(p.x(), p.y() + picture.height()/2.0f-penWidth/2));
    poly.replace(3, QPointF(p.x() - picture.width()/2.0f+penWidth/2, p.y()));

    int waypointSize = qMin(picture.width(), picture.height());
    float rad = (waypointSize/2.0f) * 0.7f * (1/sqrt(2.0f));

    // If this is not a waypoint (only the default representation)
    // or it is a waypoint, but not one where direction has no meaning
    // then draw the heading indicator
    if (!waypoint || (waypoint && (
            (waypoint->getAction() != (int)MAV_CMD_NAV_TAKEOFF) &&
            (waypoint->getAction() != (int)MAV_CMD_NAV_LAND) &&
            (waypoint->getAction() != (int)MAV_CMD_NAV_LOITER_UNLIM) &&
            (waypoint->getAction() != (int)MAV_CMD_NAV_LOITER_TO_ALT) &&
            (waypoint->getAction() != (int)MAV_CMD_NAV_LOITER_TIME) &&
            (waypoint->getAction() != (int)MAV_CMD_NAV_LOITER_TURNS) &&
            (waypoint->getAction() != (int)MAV_CMD_NAV_RETURN_TO_LAUNCH)&&
            (waypoint->getAction() != (int)MAV_CMD_DO_SET_ROI)
            )))
    {
        painter.setPen(pen1);
        painter.drawLine(p.x(), p.y(), p.x()+sin(Heading()/180.0f*M_PI) * rad, p.y()-cos(Heading()/180.0f*M_PI) * rad);
        painter.setPen(pen2);
        painter.drawLine(p.x(), p.y(), p.x()+sin(Heading()/180.0f*M_PI) * rad, p.y()-cos(Heading()/180.0f*M_PI) * rad);
    }

    if ((waypoint != NULL) && (waypoint->getAction() == (int)MAV_CMD_NAV_TAKEOFF))
    {
        // Takeoff waypoint
        int width = picture.width()-penWidth;
        int height = picture.height()-penWidth;

        painter.setPen(pen1);
        painter.drawRect(penWidth/2, penWidth/2, width, height);
        painter.setPen(pen2);
        painter.drawRect(penWidth/2, penWidth/2, width, height);

        painter.setPen(pen1);
        painter.drawRect(width*0.3, height*0.3f, width*0.6f, height*0.6f);
        painter.setPen(pen2);
        painter.drawRect(width*0.3, height*0.3f, width*0.6f, height*0.6f);
    }
    else if ((waypoint != NULL) && (waypoint->getAction() == (int)MAV_CMD_NAV_LAND))
    {
        // Landing waypoint
        int width = (picture.width())/2-penWidth;
        int height = (picture.height())/2-penWidth;
        painter.setPen(pen1);
        painter.drawEllipse(p, width, height);
        painter.drawLine(p.x()-width/2, p.y()-height/2, 2*width, 2*height);
        painter.setPen(pen2);
        painter.drawEllipse(p, width, height);
        painter.drawLine(p.x()-width/2, p.y()-height/2, 2*width, 2*height);
    }
    else if ((waypoint != NULL) && ((waypoint->getAction() == (int)MAV_CMD_NAV_LOITER_TO_ALT) || (waypoint->getAction() == (int)MAV_CMD_NAV_LOITER_UNLIM) || (waypoint->getAction() == (int)MAV_CMD_NAV_LOITER_TIME) || (waypoint->getAction() == (int)MAV_CMD_NAV_LOITER_TURNS)))
    {
        // Loiter waypoint
        int width = (picture.width()-penWidth)/2;
        int height = (picture.height()-penWidth)/2;
        painter.setPen(pen1);
        painter.drawEllipse(p, width, height);
        painter.drawPoint(p);
        painter.setPen(pen2);
        painter.drawEllipse(p, width, height);
        painter.drawPoint(p);
    }
    else if ((waypoint != NULL) && (waypoint->getAction() == (int)MAV_CMD_DO_SET_ROI))
    {
        // Region of Interest
        int width = (picture.width()-penWidth)/2;
        int height = (picture.height()-penWidth)/4;
        painter.setPen(pen1);
        painter.drawEllipse(p, width, height);
        painter.drawEllipse(p, height, height);
        painter.drawPoint(p);
        painter.setPen(pen2);
        painter.drawEllipse(p, width, height);
        painter.drawEllipse(p, height, height);
        painter.drawPoint(p);
    }
    else if ((waypoint != NULL) && (waypoint->getAction() == (int)MAV_CMD_NAV_RETURN_TO_LAUNCH))
    {
        // Return to launch waypoint
        int width = picture.width()-penWidth;
        int height = picture.height()-penWidth;
        painter.setPen(pen1);
        painter.drawRect(penWidth/2, penWidth/2, width, height);
        painter.setPen(pen2);
        painter.drawRect(penWidth/2, penWidth/2, width, height);

        QString text("R");

        painter.setPen(pen1);
        QRect rect = metrics.boundingRect(0, 0, width - 2*border, height, Qt::AlignLeft | Qt::TextWordWrap, text);
        painter.drawText(width/4, height/6, rect.width(), rect.height(),
                          Qt::AlignCenter | Qt::TextWordWrap, text);
        painter.setPen(pen2);

        font.setPixelSize(fontSize*0.85f);
        painter.setFont(font);
        painter.drawText(width/4, height/6, rect.width(), rect.height(), Qt::AlignCenter | Qt::TextWordWrap, text);
    }
    else
    {
        // Navigation waypoint
        painter.setPen(pen1);
        painter.drawPolygon(poly);
        painter.setPen(pen2);
        painter.drawPolygon(poly);
    }
}

void Waypoint2DIcon::SetShowNumber(const bool &value)
{
    shownumber=value;
    if((numberI == NULL) && value)
    {
        Q_ASSERT(numberIBG == NULL);
        numberI=new QGraphicsSimpleTextItem(this);
        numberIBG=new QGraphicsRectItem(this);
        numberIBG->setBrush(Qt::black);
        numberIBG->setOpacity(0.5);
        numberI->setZValue(3);
        numberI->setPen(QPen(QGC::colorCyan));
        numberI->setPos(5,-picture.height());
        numberIBG->setPos(5,-picture.height());
        numberI->setText(QString::number(number));
        numberIBG->setRect(numberI->boundingRect().adjusted(-2,0,1,0));
    }
    else if (!value && numberI)
    {
        delete numberI;
        numberI = NULL;
        delete numberIBG;
        numberIBG = NULL;
    }
    this->update();
}

void Waypoint2DIcon::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);

    painter->setRenderHint(QPainter::Antialiasing, true);
    painter->setRenderHint(QPainter::HighQualityAntialiasing, true);

    QPen pen = painter->pen();
    pen.setWidth(2);
    painter->drawPixmap(-picture.width()/2,-picture.height()/2,picture);

    QPen penBlack = QPen(pen);
    penBlack.setColor(Qt::black);
    penBlack.setWidth(1);


    if (this->isSelected())
    {
        pen.setColor(Qt::yellow);
        painter->drawRect(QRectF(-picture.width()/2,-picture.height()/2,picture.width()-1,picture.height()-1));
    }

    //Draw a safety radius around the wp to show the minimum distance the aircraft can be from people
    bool showSafetyRadius = true;

    if (waypoint && showSafetyRadius)
    {
        #define MAX_SAFETY_RADIUS_METERS 30.0 //When at 0 AGL
        #define MIN_SAFETY_RADIUS_METERS 15.0 //When greater than 30m AGL - This accounts for GPS and Waypoint Error


        double SafetyRadius = 0;

        if (waypoint->getAltitude() < MAX_SAFETY_RADIUS_METERS)
            {
            //Use the right angle rule X = Square Root ( R^2 - y^2 )
            SafetyRadius = sqrt((MAX_SAFETY_RADIUS_METERS * MAX_SAFETY_RADIUS_METERS) - (waypoint->getAltitude() * waypoint->getAltitude()));
            }

        if (SafetyRadius < MIN_SAFETY_RADIUS_METERS )
            {
            SafetyRadius = MIN_SAFETY_RADIUS_METERS;
            }


        QPen penDash(Qt::blue);
        penDash.setWidth(2);
        penDash.setStyle(Qt::DotLine);
        const int SafetyRadiusPixels = map->metersToPixels(SafetyRadius, Coord());

        painter->setPen(penDash);
        painter->drawEllipse(QPointF(0, 0), SafetyRadiusPixels, SafetyRadiusPixels);
        //painter->drawEllipse(QPointF(0, 0), SafetyRadiusPixels/2, SafetyRadiusPixels/2);

    }

    bool showCameraFootprint = true;

    if (waypoint && showCameraFootprint)
    {
        #define X_CAMERA_FOOTPRINT_METERS_AT_1M  1.4 //meters
        #define Y_CAMERA_FOOTPRINT_METERS_AT_1M  0.8 //meters

        #define X_CAMERA_OVERLAP  70 //percent
        #define Y_CAMERA_OVERLAP  70 //percent

        QPen penCameraFootprint(Qt::gray);
        penCameraFootprint.setWidth(2);
        penCameraFootprint.setStyle(Qt::DashDotDotLine);

        QPen penCameraOverlap(Qt::white);
        penCameraFootprint.setWidth(2);

        double X_CameraFootprintMeters = waypoint->getAltitude() * X_CAMERA_FOOTPRINT_METERS_AT_1M;
        double Y_CameraFootprintMeters = waypoint->getAltitude() * Y_CAMERA_FOOTPRINT_METERS_AT_1M;

        const int X_CameraFootprintPixels = map->metersToPixels(X_CameraFootprintMeters, Coord());
        const int Y_CameraFootprintPixels = map->metersToPixels(Y_CameraFootprintMeters, Coord());

        const int X_CameraOverlapPixels = map->metersToPixels(X_CameraFootprintMeters * (100 - X_CAMERA_OVERLAP)/100, Coord());
        const int Y_CameraOverlapPixels = map->metersToPixels(Y_CameraFootprintMeters * (100 - Y_CAMERA_OVERLAP)/100, Coord());

        painter->rotate(waypoint->getYaw());
        painter->setPen(penCameraFootprint);
        painter->drawRect((0 - X_CameraFootprintPixels),(0 - Y_CameraFootprintPixels), (X_CameraFootprintPixels*2),(Y_CameraFootprintPixels*2));

        painter->setPen(penCameraOverlap);
        painter->drawRect((0 - X_CameraOverlapPixels),(0 - Y_CameraOverlapPixels), (X_CameraOverlapPixels*2),(Y_CameraOverlapPixels*2));

    }


    if (waypoint && showAcceptanceRadius &&
           ( (waypoint->getAction() == (int)MAV_CMD_NAV_WAYPOINT)
            ||(waypoint->getAction() == (int)MAV_CMD_NAV_SPLINE_WAYPOINT)) )
    {
        QPen yellowPen = QPen(pen);
        yellowPen.setColor(Qt::yellow);
        yellowPen.setWidth(1);

        const int acceptance = map->metersToPixels(waypoint->getAcceptanceRadius(), Coord());
        if (acceptance <= 0)
            return;
        painter->setPen(penBlack);
        painter->drawEllipse(QPointF(0, 0), acceptance, acceptance);
        painter->setPen(yellowPen);
        painter->drawEllipse(QPointF(0, 0), acceptance, acceptance);
    }
    if ((waypoint) && ((waypoint->getAction() == (int)MAV_CMD_NAV_LOITER_TO_ALT) || (waypoint->getAction() == (int)MAV_CMD_NAV_LOITER_UNLIM) || (waypoint->getAction() == (int)MAV_CMD_NAV_LOITER_TIME) || (waypoint->getAction() == (int)MAV_CMD_NAV_LOITER_TURNS)))
    {
        QLOG_DEBUG() << "DRAW LOITER" ;
        QPen penDash(color);
        penDash.setWidth(1);
        //penDash.setStyle(Qt::DotLine);
        int loiter=0;
        if (waypoint->getAction() == (int)MAV_CMD_NAV_LOITER_TO_ALT)
        {loiter = map->metersToPixels(waypoint->getParam2(), Coord());}
        else
        {loiter = map->metersToPixels(waypoint->getLoiterOrbit(), Coord());}
        if (loiter > picture.width()/2)
        {
            painter->setPen(penBlack);
            painter->drawEllipse(QPointF(0, 0), loiter, loiter);
            painter->setPen(penDash);
            painter->drawEllipse(QPointF(0, 0), loiter, loiter);
        }
    }
}
