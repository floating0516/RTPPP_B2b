#include "bncsate.h"
#include <QPainter>
#include <cmath>
#include "rtkdefine.h"

static const double kPI = 3.14159265358979323846;
static const QColor kColGPS(Qt::green);
static const QColor kColGLO(Qt::magenta);
static const QColor kColGAL(Qt::red);
static const QColor kColBDS(Qt::blue);

static double computePDOPFromAzElUsedSys(const QVector<BncSatData> &sats, int sysMask)
{
    int n = 0;
    for (int i = 0; i < sats.size(); i++) {
        const BncSatData &s = sats[i];
        if (s.sys == sysMask && s.used && s.el > 0.0) n++;
    }
    if (n < 4) return 0.0;
    double Sxx=0, Syy=0, Szz=0, Sxy=0, Sxz=0, Syz=0;
    for (int i = 0; i < sats.size(); i++) {
        const BncSatData &s = sats[i];
        if (s.sys != sysMask || !s.used || s.el <= 0.0) continue;
        double el = s.el * kPI / 180.0;
        double az = s.az * kPI / 180.0;
        double sx = cos(el) * cos(az);
        double sy = cos(el) * sin(az);
        double sz = sin(el);
        Sxx += sx*sx; Syy += sy*sy; Szz += sz*sz;
        Sxy += sx*sy; Sxz += sx*sz; Syz += sy*sz;
    }
    double det = Sxx*(Syy*Szz - Syz*Syz) - Sxy*(Sxy*Szz - Syz*Sxz) + Sxz*(Sxy*Syz - Syy*Sxz);
    if (fabs(det) < 1e-12) return 0.0;
    double C11 =  (Syy*Szz - Syz*Syz);
    double C22 =  (Sxx*Szz - Sxz*Sxz);
    double C33 =  (Sxx*Syy - Sxy*Sxy);
    double pdop = sqrt((C11 + C22 + C33) / det);
    if (!std::isfinite(pdop) || pdop < 0) return 0.0;
    return pdop;
}

BncSate::BncSate(QWidget *parent) : QWidget(parent)
{
    setAutoFillBackground(true);
    QPalette palette = this->palette();
    palette.setColor(QPalette::Window, Qt::white);
    setPalette(palette);
    _tRangeSec = 900.0;
}

BncSate::~BncSate()
{
}

void BncSate::slotNewSatInfo(BncSatInfo info)
{
    QMutexLocker locker(&_mutex);
    _currentInfo = info;
    _history.append(info);
    if (_history.size() == 1) {
        _startTime = info.time;
    }
    
    // Slide window to last _tRangeSec seconds
    while (_history.size() > 0) {
        double dt = info.time - _history.first().time;
        if (dt > _tRangeSec) _history.removeFirst();
        else break;
    }
    
    update();
}

void BncSate::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    int w = width();
    int h = height();

    // Layout: 
    // Left half: Sat Num (Top), PDOP (Bottom) with 15-min time axis
    // Right half: Skyplot with azimuth degree labels

    QRect leftTopRect(0, 0, w / 2, h / 2);
    QRect leftBottomRect(0, h / 2, w / 2, h / 2);
    QRect rightRect(w / 2, 0, w / 2, h);

    drawSatNumPlot(painter, leftTopRect);
    drawDopPlot(painter, leftBottomRect);
    drawSkyPlot(painter, rightRect);
}

void BncSate::drawSkyPlot(QPainter &painter, const QRect &rect)
{
    painter.save();
    painter.translate(rect.center());
    
    int size = qMin(rect.width(), rect.height()) - 20;
    int r = size / 2;

    // Draw grid
    painter.setPen(QPen(Qt::gray, 1, Qt::DashLine));
    painter.drawEllipse(-r, -r, 2 * r, 2 * r); // 0 deg
    painter.drawEllipse(-r * 2 / 3, -r * 2 / 3, r * 4 / 3, r * 4 / 3); // 30 deg
    painter.drawEllipse(-r / 3, -r / 3, r * 2 / 3, r * 2 / 3); // 60 deg

    painter.drawLine(0, -r, 0, r);
    painter.drawLine(-r, 0, r, 0);
    painter.setPen(QPen(Qt::darkGray));
    QFont f = painter.font();
    f.setPointSize(int(f.pointSize() * 0.9));
    painter.setFont(f);
    // Azimuth labels every 30°
    for (int azDeg = 0; azDeg < 360; azDeg += 30) {
        double rr = r + 6;
        double xx = rr * sin(azDeg * kPI / 180.0);
        double yy = -rr * cos(azDeg * kPI / 180.0);
        QString text = QString::number(azDeg);
        painter.drawText(QPointF(xx - 8, yy + 4), text);
    }
    // Elevation ring labels (0,30,60)
    painter.drawText(QPointF(-10, -r - 6), "0");
    painter.drawText(QPointF(-10, -(r * 2 / 3) - 6), "30");
    painter.drawText(QPointF(-10, -(r / 3) - 6), "60");

    // Draw satellites
    QMutexLocker locker(&_mutex);
    
    // Draw trails (optional, can be expensive)
    // For now, just draw current epoch
    
    if (_history.size() > 0) {
        // Draw trails for last N epochs
        int trailLen = qMin(_history.size(), 120); // Last 2 mins approx
        for (int i = 0; i < trailLen; i++) {
            const BncSatInfo &info = _history.at(_history.size() - 1 - i);
            for (int j = 0; j < info.sats.size(); j++) {
                const BncSatData &sat = info.sats[j];
                if (sat.el <= 0) continue;
                
                double el = sat.el;
                double az = sat.az;
                
                double rr = r * (90.0 - el) / 90.0;
                double xx = rr * sin(az * kPI / 180.0);
                double yy = -rr * cos(az * kPI / 180.0);
                
                QColor col = Qt::gray;
                if (sat.sys == SYS_GPS) col = kColGPS;
                else if (sat.sys == SYS_GLO) col = kColGLO;
                else if (sat.sys == SYS_GAL) col = kColGAL;
                else if (sat.sys == SYS_CMP) col = kColBDS;
                
                col.setAlpha(100 - i * 100 / trailLen);
                painter.setPen(Qt::NoPen);
                painter.setBrush(col);
                painter.drawEllipse(QPointF(xx, yy), 2, 2);
            }
        }
    }

    // Draw current sats
    for (int i = 0; i < _currentInfo.sats.size(); i++) {
        const BncSatData &sat = _currentInfo.sats[i];
        if (sat.el <= 0) continue;
        
        double el = sat.el;
        double az = sat.az;
        
        double rr = r * (90.0 - el) / 90.0;
        double xx = rr * sin(az * kPI / 180.0);
        double yy = -rr * cos(az * kPI / 180.0);
        
        QColor col = Qt::gray;
        if (sat.sys == SYS_GPS) col = kColGPS;
        else if (sat.sys == SYS_GLO) col = kColGLO;
        else if (sat.sys == SYS_GAL) col = kColGAL;
        else if (sat.sys == SYS_CMP) col = kColBDS;
        
        painter.setPen(Qt::black);
        painter.setBrush(col);
        painter.drawEllipse(QPointF(xx, yy), 5, 5);
        
        painter.drawText(QPointF(xx + 8, yy), QString::number(sat.prn));
    }

    painter.restore();
}

void BncSate::drawSatNumPlot(QPainter &painter, const QRect &rect)
{
    painter.save();
    painter.setClipRect(rect);
    
    // Draw background and frame
    painter.fillRect(rect, Qt::white);
    painter.setPen(Qt::black);
    QRect area = rect.adjusted(40, 20, -20, -30);
    painter.drawRect(area);
    
    QFont f0 = painter.font();
    painter.setPen(Qt::blue);
    painter.drawText(area.topLeft() + QPoint(6, 16), "Sat Num");
    painter.setPen(Qt::black);

    QMutexLocker locker(&_mutex);
    if (_history.isEmpty()) {
        painter.restore();
        return;
    }

    // Determine time window
    bncTime lastT = _history.back().time;
    bncTime startT = lastT - _tRangeSec;
    // Determine y-scale
    int maxSat = 0;
    for (const auto &info : _history) {
        if (info.numSat > maxSat) maxSat = info.numSat;
    }
    int yMax = qMax(10, ((maxSat + 4) / 5) * 5);

    int w = area.width();
    int h = area.height();
    
    QVector<QPointF> pts;
    painter.setPen(Qt::NoPen);
    painter.setBrush(Qt::blue);
    int r = 3;
    for (int i = 0; i < _history.size(); i++) {
        const BncSatInfo &inf = _history[i];
        double dt = inf.time - startT;
        if (dt < 0 || dt > _tRangeSec) continue;
        double x = area.left() + dt / _tRangeSec * w;
        double y = area.bottom() - (inf.numSat / (double)yMax) * h;
        pts.push_back(QPointF(x, y));
        painter.drawEllipse(QPointF(x, y), r, r);
    }
    painter.setPen(QPen(Qt::blue, 2));
    if (pts.size() > 1) painter.drawPolyline(pts.constData(), pts.size());

    painter.setPen(Qt::NoPen);
    int rSys = 2;
    QVector<QPointF> gPts, rPts, ePts, cPts;
    QColor colG = kColGPS;
    QColor colR = kColGLO;
    QColor colE = kColGAL;
    QColor colC = kColBDS;
    painter.setBrush(colG);
    int dxG = -3, dxR = -1, dxE = 1, dxC = 3;
    for (int i = 0; i < _history.size(); i++) {
        const BncSatInfo &inf = _history[i];
        double dt = inf.time - startT;
        if (dt < 0 || dt > _tRangeSec) continue;
        int cnt = 0;
        for (int k = 0; k < inf.sats.size(); k++) {
            const BncSatData &s = inf.sats[k];
            if (s.sys == SYS_GPS && s.used && s.el > 0.0) cnt++;
        }
        double x = area.left() + dt / _tRangeSec * w + dxG;
        double y = area.bottom() - (cnt / (double)yMax) * h;
        gPts.push_back(QPointF(x, y));
        painter.drawEllipse(QPointF(x, y), rSys, rSys);
    }
    painter.setBrush(colR);
    for (int i = 0; i < _history.size(); i++) {
        const BncSatInfo &inf = _history[i];
        double dt = inf.time - startT;
        if (dt < 0 || dt > _tRangeSec) continue;
        int cnt = 0;
        for (int k = 0; k < inf.sats.size(); k++) {
            const BncSatData &s = inf.sats[k];
            if (s.sys == SYS_GLO && s.used && s.el > 0.0) cnt++;
        }
        double x = area.left() + dt / _tRangeSec * w + dxR;
        double y = area.bottom() - (cnt / (double)yMax) * h;
        rPts.push_back(QPointF(x, y));
        painter.drawEllipse(QPointF(x, y), rSys, rSys);
    }
    painter.setBrush(colE);
    for (int i = 0; i < _history.size(); i++) {
        const BncSatInfo &inf = _history[i];
        double dt = inf.time - startT;
        if (dt < 0 || dt > _tRangeSec) continue;
        int cnt = 0;
        for (int k = 0; k < inf.sats.size(); k++) {
            const BncSatData &s = inf.sats[k];
            if (s.sys == SYS_GAL && s.used && s.el > 0.0) cnt++;
        }
        double x = area.left() + dt / _tRangeSec * w + dxE;
        double y = area.bottom() - (cnt / (double)yMax) * h;
        ePts.push_back(QPointF(x, y));
        painter.drawEllipse(QPointF(x, y), rSys, rSys);
    }
    painter.setBrush(colC);
    for (int i = 0; i < _history.size(); i++) {
        const BncSatInfo &inf = _history[i];
        double dt = inf.time - startT;
        if (dt < 0 || dt > _tRangeSec) continue;
        int cnt = 0;
        for (int k = 0; k < inf.sats.size(); k++) {
            const BncSatData &s = inf.sats[k];
            if (s.sys == SYS_CMP && s.used && s.el > 0.0) cnt++;
        }
        double x = area.left() + dt / _tRangeSec * w + dxC;
        double y = area.bottom() - (cnt / (double)yMax) * h;
        cPts.push_back(QPointF(x, y));
        painter.drawEllipse(QPointF(x, y), rSys, rSys);
    }
    painter.setPen(QPen(colG, 1)); if (gPts.size() > 1) painter.drawPolyline(gPts.constData(), gPts.size());
    painter.setPen(QPen(colR, 1)); if (rPts.size() > 1) painter.drawPolyline(rPts.constData(), rPts.size());
    painter.setPen(QPen(colE, 1)); if (ePts.size() > 1) painter.drawPolyline(ePts.constData(), ePts.size());
    painter.setPen(QPen(colC, 1)); if (cPts.size() > 1) painter.drawPolyline(cPts.constData(), cPts.size());

    int baseX = area.left() + 70;
    int baseY = area.top() + 16;
    painter.setPen(colG); painter.drawText(baseX + 0,  baseY, "GPS");
    painter.setPen(colR); painter.drawText(baseX + 40, baseY, "GLO");
    painter.setPen(colE); painter.drawText(baseX + 80, baseY, "GAL");
    painter.setPen(colC); painter.drawText(baseX + 120, baseY, "BDS");
    painter.setPen(QPen(Qt::gray, 1, Qt::DashLine));
    for (int i = 0; i <= 5; i++) {
        double y = area.bottom() - (i / 5.0) * h;
        painter.drawLine(area.left(), y, area.right(), y);
    }
    // Minute ticks and labels
    QFont font = painter.font();
    int ww = QFontMetrics(font).width('w');
    for (int m = 0; m <= 15; m++) {
        double dt = m * 60.0;
        double x = area.left() + dt / _tRangeSec * w;
        painter.drawLine(int(x), area.bottom(), int(x), area.bottom() + 4);
        if (m % 5 == 0) {
            painter.drawText(int(x - ww), area.bottom() + 16, QString::number(m) + "m");
        }
    }
    // Y-axis labels
    painter.setPen(Qt::black);
    for (int i = 0; i <= 5; i++) {
        int val = int(i * yMax / 5.0);
        double y = area.bottom() - (val / (double)yMax) * h;
        painter.drawText(area.left() - 30, int(y + 4), QString::number(val));
    }
    
    painter.restore();
}

void BncSate::drawDopPlot(QPainter &painter, const QRect &rect)
{
    painter.save();
    painter.setClipRect(rect);
    
    // Draw background and frame
    painter.fillRect(rect, Qt::white);
    painter.setPen(Qt::black);
    QRect area = rect.adjusted(40, 20, -20, -30);
    painter.drawRect(area);
    
    painter.setPen(Qt::red);
    painter.drawText(area.topLeft() + QPoint(6, 16), "PDOP");
    painter.setPen(Qt::black);

    QMutexLocker locker(&_mutex);
    if (_history.isEmpty()) {
        painter.restore();
        return;
    }

    bncTime lastT = _history.back().time;
    bncTime startT = lastT - _tRangeSec;
    double maxDop = 0;
    for (const auto &info : _history) {
        if (info.pdop > maxDop) maxDop = info.pdop;
        double g = computePDOPFromAzElUsedSys(info.sats, SYS_GPS);
        double r = computePDOPFromAzElUsedSys(info.sats, SYS_GLO);
        double e = computePDOPFromAzElUsedSys(info.sats, SYS_GAL);
        double c = computePDOPFromAzElUsedSys(info.sats, SYS_CMP);
        if (g > maxDop) maxDop = g;
        if (r > maxDop) maxDop = r;
        if (e > maxDop) maxDop = e;
        if (c > maxDop) maxDop = c;
    }
    double yMax = qMax(5.0, ceil(maxDop * 2.0) / 2.0);

    int w = area.width();
    int h = area.height();
    
    QVector<QPointF> dpts;
    painter.setPen(Qt::NoPen);
    painter.setBrush(Qt::red);
    int r = 3;
    for (int i = 0; i < _history.size(); i++) {
        const BncSatInfo &inf = _history[i];
        double dt = inf.time - startT;
        if (dt < 0 || dt > _tRangeSec) continue;
        double x = area.left() + dt / _tRangeSec * w;
        double y = area.bottom() - (inf.pdop / yMax) * h;
        dpts.push_back(QPointF(x, y));
        painter.drawEllipse(QPointF(x, y), r, r);
    }
    painter.setPen(QPen(Qt::red, 2));
    if (dpts.size() > 1) painter.drawPolyline(dpts.constData(), dpts.size());

    int rSys = 2;
    QColor colG = kColGPS;
    QColor colR = kColGLO;
    QColor colE = kColGAL;
    QColor colC = kColBDS;
    QVector<QPointF> gPts, rPts, ePts, cPts;
    painter.setPen(Qt::NoPen);
    painter.setBrush(colG);
    for (int i = 0; i < _history.size(); i++) {
        const BncSatInfo &inf = _history[i];
        double dt = inf.time - startT;
        if (dt < 0 || dt > _tRangeSec) continue;
        double val = computePDOPFromAzElUsedSys(inf.sats, SYS_GPS);
        if (val <= 0.0) continue;
        double x = area.left() + dt / _tRangeSec * w;
        double y = area.bottom() - (val / yMax) * h;
        gPts.push_back(QPointF(x, y));
        painter.drawEllipse(QPointF(x, y), rSys, rSys);
    }
    painter.setBrush(colR);
    for (int i = 0; i < _history.size(); i++) {
        const BncSatInfo &inf = _history[i];
        double dt = inf.time - startT;
        if (dt < 0 || dt > _tRangeSec) continue;
        double val = computePDOPFromAzElUsedSys(inf.sats, SYS_GLO);
        if (val <= 0.0) continue;
        double x = area.left() + dt / _tRangeSec * w;
        double y = area.bottom() - (val / yMax) * h;
        rPts.push_back(QPointF(x, y));
        painter.drawEllipse(QPointF(x, y), rSys, rSys);
    }
    painter.setBrush(colE);
    for (int i = 0; i < _history.size(); i++) {
        const BncSatInfo &inf = _history[i];
        double dt = inf.time - startT;
        if (dt < 0 || dt > _tRangeSec) continue;
        double val = computePDOPFromAzElUsedSys(inf.sats, SYS_GAL);
        if (val <= 0.0) continue;
        double x = area.left() + dt / _tRangeSec * w;
        double y = area.bottom() - (val / yMax) * h;
        ePts.push_back(QPointF(x, y));
        painter.drawEllipse(QPointF(x, y), rSys, rSys);
    }
    painter.setBrush(colC);
    for (int i = 0; i < _history.size(); i++) {
        const BncSatInfo &inf = _history[i];
        double dt = inf.time - startT;
        if (dt < 0 || dt > _tRangeSec) continue;
        double val = computePDOPFromAzElUsedSys(inf.sats, SYS_CMP);
        if (val <= 0.0) continue;
        double x = area.left() + dt / _tRangeSec * w;
        double y = area.bottom() - (val / yMax) * h;
        cPts.push_back(QPointF(x, y));
        painter.drawEllipse(QPointF(x, y), rSys, rSys);
    }
    painter.setPen(QPen(colG, 1)); if (gPts.size() > 1) painter.drawPolyline(gPts.constData(), gPts.size());
    painter.setPen(QPen(colR, 1)); if (rPts.size() > 1) painter.drawPolyline(rPts.constData(), rPts.size());
    painter.setPen(QPen(colE, 1)); if (ePts.size() > 1) painter.drawPolyline(ePts.constData(), ePts.size());
    painter.setPen(QPen(colC, 1)); if (cPts.size() > 1) painter.drawPolyline(cPts.constData(), cPts.size());
    painter.setPen(QPen(Qt::gray, 1, Qt::DashLine));
    for (int i = 0; i <= 5; i++) {
        double y = area.bottom() - (i / 5.0) * h;
        painter.drawLine(area.left(), y, area.right(), y);
    }
    // Minute ticks and labels
    QFont font = painter.font();
    int ww = QFontMetrics(font).width('w');
    for (int m = 0; m <= 15; m++) {
        double dt = m * 60.0;
        double x = area.left() + dt / _tRangeSec * w;
        painter.drawLine(int(x), area.bottom(), int(x), area.bottom() + 4);
        if (m % 5 == 0) {
            painter.drawText(int(x - ww), area.bottom() + 16, QString::number(m) + "m");
        }
    }
    // Y-axis labels
    painter.setPen(Qt::black);
    for (int i = 0; i <= 5; i++) {
        double val = i * yMax / 5.0;
        double y = area.bottom() - (val / yMax) * h;
        painter.drawText(area.left() - 34, int(y + 4), QString::number(val, 'f', 1));
    }
    
    painter.restore();
}

int BncSate::timeToX(const bncTime &t, const QRect &rect, const bncTime &tStart) const
{
    double dt = t - tStart;
    double w = rect.width();
    return int(rect.left() + qMax(0.0, qMin(_tRangeSec, dt)) / _tRangeSec * w);
}

int BncSate::valueToY(double v, const QRect &rect, double vMin, double vMax) const
{
    double h = rect.height();
    double rate = (v - vMin) / (vMax - vMin);
    return int(rect.bottom() - qBound(0.0, rate, 1.0) * h);
}
