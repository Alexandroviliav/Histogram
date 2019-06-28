#ifndef INTENSITYHISTOGRAM_H
#define INTENSITYHISTOGRAM_H

#include <QObject>
#include <QDebug>
#include "segment.h"
#include <QMouseEvent>
#include <qcustomplot.h>
#include <QtAlgorithms>
#include "qxtspanslider.h"
#include "qxtspanslider_p.h"
#include "imageacquisition.h"

#define EPSILON 1.0e-5
#define RESOLUTION 32
#define CONTINIOUSLY 1

class IntensityHistogram : public QObject
{
    Q_OBJECT
public:
    explicit            IntensityHistogram(QObject *parent = nullptr);
    QCustomPlot*        getPointerToPlot();
    QxtSpanSlider*      getPointerToSlider();
    QPushButton*        getPointerToInversionButton();
    QPushButton*        getPointerToClarityButton();
    QPushButton*        getPointerToBaCButton();
    void                setDataPlot(int &width, int &height, ushort *data);
    void                setDataPlotFromFile();
    QVector<int>        getXValues();
    QVector<double>     getYValues();
    QVector<double>     getVectorPointX();
    QVector<double>     getVectorPointY();
    void                setXYValues();
    void                clarityIsInvisible();
    void                clearPlot();
    bool                getBaCFlag();
    bool                getClarityFlag();
    void                setAutoContrast(bool flag);
    bool                getAutoContrast();

private:
    const int axisLenghtX   = 65535;
    int imprecision         = 5;    // Задает размер точки
    const int imprecisionY  = 65535;
    int offsetMargin        = 1000;
    int offsetMarginY       = 2500;

    const double percentAutoContrast = 0.1; // Процент для определения границы автоконтраста

    QCustomPlot     *plot;
    QCPCurve        *verticalLine;  // Объявляем объект для вертикальной линии
    QCPItemTracer   *tracer;        // Трасировщик по точкам графика
    QxtSpanSlider   *slider;        // Слайдер
    QPushButton     *inversionButton;
    QPushButton     *clarityButton;
    QPushButton     *bacButton;

    QVector<double>     lineXPrevious, lineYPrevious;
    QVector<double>     vectorPointX, vectorPointY;     // Метки
    QVector<QPointF>    vectorCoordPoint;               // Координаты меток для перемещения
    QList<double>       pointsForLineX, pointsForLineY; // Линия
    QVector<double>     lineX, lineY;
    QVector<int>        xValues;
    QVector<double>     yValues;

    bool lineChangeState    = false;
    bool graphChangeState   = false;
    bool intersectionState  = false;
    bool isPointCaptured    = false;
    bool isZoom             = false;
    bool isNonDefaultSet    = false;
    bool isDeletePoint      = true;
    bool isBoundaryPointCap = false;
    bool isNonBoudaryPoint  = false;

    // Флаги передачи яркости - контрстности или прозрачности
    bool isBaC              = true;
    bool isClarity          = false;

    // Флаг для установки автоконтраста
    bool isAutoContrast     = true;
    int lowerValueForBoundaries = 0;
    int upperValueForBoundaries = INTENSITY_MAX;

    double coordXPrevious, coordYPrevious;
    double conversionFactorY    = 1;
    double previousRangeY       = 65535;
    double percentForDeviation  = 100;
    int currentPointX           = 0;
    int nextPointX              = 1;
    int itemNumberPrevious      = 0;
    int minCoordinateX          = 0;
    int maxCoordinateX          = 65535;
    int minCoordinateY          = 0;
    int maxCoordinateY          = 65535;
    int iteratorPrevious        = 0;

    // Данные линии прозрачности
    QVector<double>     lineXPreviousClarity, lineYPreviousClarity;
    QVector<double>     vectorPointXClarity, vectorPointYClarity;
    QVector<QPointF>    vectorCoordPointClarity;
    QList<double>       pointsForLineXClarity, pointsForLineYClarity;
    QVector<double>     lineXClarity, lineYClarity;
    QVector<int>        xValuesClarity;
    QVector<double>     yValuesClarity;
    int itemNumberPreviousClarity   = 0;
    int iteratorPreviousClarity     = 0;

    // Данные линии яркости и контрастности
    QVector<double>     lineXPreviousBaC, lineYPreviousBaC;
    QVector<double>     vectorPointXBaC, vectorPointYBaC;
    QVector<QPointF>    vectorCoordPointBaC;
    QList<double>       pointsForLineXBaC, pointsForLineYBaC;
    QVector<double>     lineXBaC, lineYBaC;
    QVector<int>        xValuesBaC;
    QVector<double>     yValuesBaC;
    int itemNumberPreviousBaC   = 0;
    int iteratorPreviousBaC     = 0;

    int     itemSearch();
    bool    isThreePoints();
    bool    calculateSpline(const std::vector<Point2D> &, std::vector<Segment> &);
    bool    isLineGoBeyond();
    bool    isPointsGoBeyond();
    void    setTracer();
    void    setDefaultGraphSettings();
    void    setDefaultSettingsForLines();
    void    addSlider();
    void    addInversionButton();
    void    addClarityButton();
    void    addBaCButton();
    void    savePreviousState();
    void    changeLine(double x, double y);
    void    redrawGraph(QMouseEvent *event);
    void    xCrossesPoint(double &coordX, double &coordY);
    void    addPointGraph();
    void    addPoint(double x, double y);
    void    clearPoint();
    void    findItemNumberPrevious();
    void    recalculationQPoint();
    void    recalculateLine();
    void    clearData();
    void    clearLine();
    void    doAutoContrast(QVector<double>& vectorY, double maxY);

private slots:
    void slotMousePress(QMouseEvent *event);
    void slotMouseMove(QMouseEvent *event);
    void slotMouseRelease(QMouseEvent *event);
    void slotMouseDoubleClick(QMouseEvent *event);
    void slotInversion();
    void slotClarity();
    void slotBaC();
    void changeBoundaries(int lower, int upper);
    void slotSmallerSize();
    void slotWheelToSlider();
    void slotMouseWheel(QWheelEvent *event);
    void on_pushButton_clicked();

signals:
    void signalChangedLine(QVector<double>);
    void transferPoint(QVector<double> x, QVector<double> y);

public slots:
};

#endif // INTENSITYHISTOGRAM_H
