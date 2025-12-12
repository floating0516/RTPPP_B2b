#ifndef BNCSATE_H
#define BNCSATE_H

#include <QWidget>
#ifdef QT5_SUPPORT
#include <QtWidgets>
#else
#include <QtGui>
#endif

#include "bnctime.h"

struct BncSatData {
    int prn;
    int sys; // 1:GPS, 2:GLO, 4:GAL, 8:BDS (using RTKLIB convention or simple char)
    double az; // degrees
    double el; // degrees
    bool used; // if used in solution
    int snr;   // signal strength
};

struct BncSatInfo {
    QByteArray staID;
    bncTime time;
    double pdop; 
    int numSat;
    QVector<BncSatData> sats;
};

Q_DECLARE_METATYPE(BncSatData)
Q_DECLARE_METATYPE(BncSatInfo)

class BncSate : public QWidget
{
    Q_OBJECT
public:
    explicit BncSate(QWidget *parent = 0);
    ~BncSate();

public slots:
    void slotNewSatInfo(BncSatInfo info);

protected:
    void paintEvent(QPaintEvent *event);

private:
    void drawSkyPlot(QPainter &painter, const QRect &rect);
    void drawSatNumPlot(QPainter &painter, const QRect &rect);
    void drawDopPlot(QPainter &painter, const QRect &rect);
    int timeToX(const bncTime &t, const QRect &rect, const bncTime &tStart) const;
    int valueToY(double v, const QRect &rect, double vMin, double vMax) const;

    QList<BncSatInfo> _history;
    BncSatInfo _currentInfo;
    QMutex _mutex;
    double _tRangeSec;
    bncTime _startTime;
};

#endif // BNCSATE_H
