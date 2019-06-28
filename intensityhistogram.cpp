#include "intensityhistogram.h"

IntensityHistogram::IntensityHistogram(QObject *parent) : QObject(parent)
{
    plot = new QCustomPlot();

    // Подключаем сигналы событий мыши от полотна графика к слотам для их обработки
    connect(plot, &QCustomPlot::mousePress,         this, &IntensityHistogram::slotMousePress);
    connect(plot, &QCustomPlot::mouseMove,          this, &IntensityHistogram::slotMouseMove);
    connect(plot, &QCustomPlot::mouseRelease,       this, &IntensityHistogram::slotMouseRelease);
    connect(plot, &QCustomPlot::smallerSize,        this, &IntensityHistogram::slotSmallerSize);
    connect(plot, &QCustomPlot::smallerSize,        this, &IntensityHistogram::slotWheelToSlider);
    connect(plot, &QCustomPlot::mouseWheel,         this, &IntensityHistogram::slotMouseWheel);
    connect(plot, &QCustomPlot::mouseDoubleClick,   this, &IntensityHistogram::slotMouseDoubleClick);

    isBaC = true;
    isClarity = false;

    setDefaultGraphSettings();
    addSlider();
    addInversionButton();
    addClarityButton();
    addBaCButton();
}

QCustomPlot *IntensityHistogram::getPointerToPlot()
{
    return plot;
}

QxtSpanSlider *IntensityHistogram::getPointerToSlider()
{
    return slider;
}

QPushButton *IntensityHistogram::getPointerToInversionButton()
{
    return inversionButton;
}

QPushButton *IntensityHistogram::getPointerToClarityButton()
{
    return clarityButton;
}

QPushButton *IntensityHistogram::getPointerToBaCButton()
{
    return bacButton;
}

void IntensityHistogram::setDataPlot(int &width, int &height, ushort *data)
{
    offsetMargin = abs(((plot->xAxis->pixelToCoord(imprecision)) - plot->xAxis->pixelToCoord(imprecision * 2)));
    offsetMarginY = abs(2 * ((plot->yAxis->pixelToCoord(imprecision)) - plot->yAxis->pixelToCoord(imprecision * 2)));

    ImageAcquisition acquisition;
    acquisition.setParameters(width, height, data);

    // Отрисовать нужно график по нашим двум массивам x и y
    plot->graph(2)->setData(acquisition.getVectorX(), acquisition.getVectorY());

    QVector<double> yValue = acquisition.getVectorY();
    double maxY = acquisition.getMaxY();
    doAutoContrast(yValue, maxY);

    maxCoordinateY = acquisition.getMaxY() * (INTENSITY_MAX / acquisition.getMaxY());
    minCoordinateY = acquisition.getMinY();

    conversionFactorY = maxCoordinateY / previousRangeY;
    previousRangeY = maxCoordinateY;

    for (int i = 0; i < vectorPointY.size(); i++)
    {
        vectorPointY[i] = vectorPointY[i] * conversionFactorY;
    }

    for (int i = 0; i < pointsForLineY.size(); i++)
    {
        pointsForLineY[i] = pointsForLineY[i] * conversionFactorY;
    }

    recalculateLine();

    plot->graph(0)->setData(lineX,lineY);
    plot->graph(1)->setData(vectorPointX,vectorPointY);

    plot->yAxis->setRange(minCoordinateY - offsetMarginY, maxCoordinateY + offsetMarginY);
    if (!isAutoContrast) {
       changeBoundaries(slider->lowerValue(), slider->upperValue());
    }
    else {
        changeBoundaries(lowerValueForBoundaries, upperValueForBoundaries);
    }
}

void IntensityHistogram::setDataPlotFromFile()
{
    offsetMargin = abs(((plot->xAxis->pixelToCoord(imprecision)) - plot->xAxis->pixelToCoord(imprecision * 2)));
    offsetMarginY = abs(2 * ((plot->yAxis->pixelToCoord(imprecision)) - plot->yAxis->pixelToCoord(imprecision * 2)));

    // Данные для гистограммы взяты из файла
    ImageAcquisition acquisition;
    acquisition.readImageFromFile();

    plot->graph(2)->setData(acquisition.getVectorX(), acquisition.getVectorY());

    QVector<double> yValue = acquisition.getVectorY();
    double maxY = acquisition.getMaxY();
    doAutoContrast(yValue, maxY);

    maxCoordinateY = acquisition.getMaxY() * (INTENSITY_MAX / acquisition.getMaxY());
    minCoordinateY = acquisition.getMinY();

    conversionFactorY = maxCoordinateY / previousRangeY;
    previousRangeY = maxCoordinateY;

    for (int i = 0; i < vectorPointY.size(); i++)
    {
        vectorPointY[i] = vectorPointY[i] * conversionFactorY;
    }

    for (int i = 0; i < pointsForLineY.size(); i++)
    {
        pointsForLineY[i] = pointsForLineY[i] * conversionFactorY;
    }

    recalculateLine();

    plot->graph(0)->setData(lineX,lineY);
    plot->graph(1)->setData(vectorPointX,vectorPointY);

    plot->yAxis->setRange(minCoordinateY - offsetMarginY, maxCoordinateY + offsetMarginY);
    if (!isAutoContrast) {
       changeBoundaries(slider->lowerValue(), slider->upperValue());
    }
    else {
        changeBoundaries(lowerValueForBoundaries, upperValueForBoundaries);
    }
}

QVector<int> IntensityHistogram::getXValues()
{
    return xValues;
}

QVector<double> IntensityHistogram::getYValues()
{
    return yValues;
}

QVector<double> IntensityHistogram::getVectorPointY()
{
    return vectorPointY;
}

QVector<double> IntensityHistogram::getVectorPointX()
{
    return vectorPointX;
}

void IntensityHistogram::slotMousePress(QMouseEvent *event)
{
    // Определяем координату X на графике, где был произведён клик мышью
    double coordX = plot->xAxis->pixelToCoord(event->pos().x());
    double coordY = plot->yAxis->pixelToCoord(event->pos().y());

    offsetMargin = abs(((plot->xAxis->pixelToCoord(imprecision)) - plot->xAxis->pixelToCoord(imprecision * 2)));
    offsetMarginY = abs(2 * ((plot->yAxis->pixelToCoord(imprecision)) - plot->yAxis->pixelToCoord(imprecision * 2)));

    // Правая кнопка мыши
    if (event->buttons() == Qt::RightButton) {
        if (vectorCoordPoint.size() == 2) {
            return;
        }
        for (int i = 2; i < vectorPointX.size(); i++)
        {
            if ((abs(plot->xAxis->coordToPixel(coordX) -
                    vectorCoordPoint.at(i).x()) <= imprecision) &&
                    (abs(plot->yAxis->coordToPixel(coordY) -
                    vectorCoordPoint.at(i).y()) <= imprecision)) {
                double x = vectorPointX.at(i);
                vectorPointX.remove(i);
                vectorPointY.remove(i);
                vectorCoordPoint.remove(i);

                for (int j = 0; j < pointsForLineX.size() - 1; j++)
                {
                    if (pointsForLineX.at(j) == x) {
                        pointsForLineX.removeAt(j);
                        pointsForLineY.removeAt(j);
                        itemNumberPrevious--;
                        isThreePoints();
                    }
                }
            }
        }
        isPointsGoBeyond();

        std::vector<Point2D> testValues;
        std::vector<Segment> spline;
        Point2D p;

        for (int i = 0; i < pointsForLineX.size() - 1; i++)
        {
            testValues.push_back(Point2D(pointsForLineX[i], pointsForLineY[i]));
        }

        calculateSpline(testValues, spline);

        QVector<double> lineXChanged, lineYChanged;

        for (auto s : spline)
        {
            for (int i = 0; i < RESOLUTION; ++i)
            {
                s.calc((double)i / (double)RESOLUTION, p);
                lineXChanged.push_back(p.x);
                lineYChanged.push_back(p.y);
            }
        }

        lineX = lineXChanged;
        lineY = lineYChanged;

        isLineGoBeyond();

        plot->graph(0)->setData(lineX,lineY);
        plot->graph(1)->setData(vectorPointX,vectorPointY);
    }

    // Левая кнопка мыши
    else if (event->buttons() == Qt::LeftButton) {
        // Подготавливаем координаты по оси X для переноса вертикальной линии
        QVector<double> x(2), y(2);
        x[0] = coordX;
        y[0] = -50;
        x[1] = coordX;
        y[1] = maxCoordinateY + offsetMarginY;

        // Устанавливаем новые координаты
        verticalLine->setData(x,y);

        // По координате X клика мыши определим ближайшие координаты для трассировщика
        tracer->setGraphKey(coordX);

        if (abs(coordY - tracer->position->value()) <= imprecisionY) {
            lineChangeState = true;
            savePreviousState();
            changeLine(coordX, coordY);
        }
        if (lineChangeState) {
            lineX = lineXPrevious;
            lineY = lineYPrevious;
            savePreviousState();
            changeLine(coordX, coordY);
            redrawGraph(event);
        }
    }
    plot->graph(0)->setData(lineX,lineY);
    setXYValues();
    emit signalChangedLine(this->getYValues());
    emit transferPoint(this->getVectorPointX(), this->getVectorPointY());
    plot->replot();
}

void IntensityHistogram::slotMouseMove(QMouseEvent *event)
{
    if(QApplication::mouseButtons()) slotMousePress(event);
}

void IntensityHistogram::slotMouseRelease(QMouseEvent *event)
{
    lineChangeState     = false;
    graphChangeState    = false;
    intersectionState   = false;
    isPointCaptured     = false;
    isBoundaryPointCap  = false;
    isNonBoudaryPoint   = false;
}

void IntensityHistogram::slotMouseDoubleClick(QMouseEvent *event)
{
    // По двойному клику левой кнопки график в начальное состояние
    if ((event->buttons() == Qt::LeftButton) ||
        (event->buttons() == Qt::MiddleButton)){
        return;
    }

    offsetMargin = abs(((plot->xAxis->pixelToCoord(imprecision)) - plot->xAxis->pixelToCoord(imprecision * 2)));
    offsetMarginY = abs(2 * ((plot->yAxis->pixelToCoord(imprecision)) - plot->yAxis->pixelToCoord(imprecision * 2)));

    std::vector<Point2D> testValues;
    std::vector<Segment> spline;
    Point2D p;

    clearData();

    pointsForLineX.push_back(0);
    pointsForLineX.push_back(65535);
    pointsForLineX.push_back(65536);

    pointsForLineY.push_back(0);
    pointsForLineY.push_back(65535);
    pointsForLineY.push_back(65535.001);

    testValues.push_back(Point2D(0, 0));
    testValues.push_back(Point2D(65535, 65535));
    testValues.push_back(Point2D(65536, 65536));

    vectorPointX.push_back(0);
    vectorPointX.push_back(65535);

    QPointF firstPoint(plot->xAxis->coordToPixel(0),plot->yAxis->coordToPixel(0));
    QPointF secondPoint(plot->xAxis->coordToPixel(65535),plot->yAxis->coordToPixel(65535));
    vectorCoordPoint.push_back(firstPoint);
    vectorCoordPoint.push_back(secondPoint);

    vectorPointY.push_back(0);
    vectorPointY.push_back(65535);

    xValues.resize(65536);
    yValues.resize(65536);

    slider->setLowerPosition(static_cast<int>(0));
    slider->setUpperPosition(static_cast<int>(99));

    calculateSpline(testValues, spline);

    for (auto s : spline)
    {
        for (int i = 0; i < RESOLUTION; ++i)
        {
            s.calc((double)i / (double)RESOLUTION, p);
            lineX.push_back(p.x);
            lineY.push_back(p.y);
        }
    }

    plot->xAxis->setRange(-offsetMargin, 65536 + offsetMargin);
    plot->yAxis->setRange(-offsetMarginY, maxCoordinateY + offsetMarginY);

    recalculationQPoint();

    plot->graph(0)->setData(lineX,lineY);
    plot->graph(1)->setData(vectorPointX,vectorPointY);

    setXYValues();
    emit signalChangedLine(this->getYValues());
    emit transferPoint(this->getVectorPointX(), this->getVectorPointY());

    plot->replot();
}

void IntensityHistogram::slotInversion()
{
    // Инвертирование значений графика
    const int maxIntensity = 65535;

    for (int i = 0; i < pointsForLineY.size(); i++)
    {
        pointsForLineY[i] = abs(maxIntensity - pointsForLineY[i]);
    }

    for (int i = 0; i < vectorPointY.size(); i++)
    {
        vectorPointY[i] = abs(maxIntensity - vectorPointY[i]);
    }

    for (int i = 0; i < vectorCoordPoint.size(); i++)
    {
        double y = abs(maxIntensity - vectorCoordPoint[i].y());
        double x = vectorCoordPoint[i].x();
        QPointF point(x,y);
        vectorCoordPoint[i] = point;
    }

    recalculateLine();
    isLineGoBeyond();

    plot->graph(0)->setData(lineX,lineY);
    plot->graph(1)->setData(vectorPointX,vectorPointY);

    setXYValues();
    emit signalChangedLine(this->getYValues());
    emit transferPoint(this->getVectorPointX(), this->getVectorPointY());

    plot->replot();
}

void IntensityHistogram::slotClarity()
{
    isBaC = false;
    isClarity = true;
    // Управление графиком передается графиком регулировки прозрачности
    qDebug() << "Прозрачноть";
    isDeletePoint = true;

    lineXPreviousBaC        = lineXPrevious;
    lineYPreviousBaC        = lineYPrevious;
    vectorPointXBaC         = vectorPointX;
    vectorPointYBaC         = vectorPointY;
    vectorCoordPointBaC     = vectorCoordPoint;
    pointsForLineXBaC       = pointsForLineX;
    pointsForLineYBaC       = pointsForLineY;
    lineXBaC                = lineX;
    lineYBaC                = lineY;
    xValuesBaC              = xValues;
    yValuesBaC              = yValues;

    plot->graph(3)->setData(lineX,lineY);
    plot->graph(4)->setData(vectorPointX,vectorPointY);

    itemNumberPreviousBaC = itemNumberPrevious;
    iteratorPreviousBaC   = iteratorPrevious;

    lineXPrevious    = lineXPreviousClarity;
    lineYPrevious    = lineYPreviousClarity;
    vectorPointX     = vectorPointXClarity;
    vectorPointY     = vectorPointYClarity;
    vectorCoordPoint = vectorCoordPointClarity;
    pointsForLineX   = pointsForLineXClarity;
    pointsForLineY   = pointsForLineYClarity;
    lineX            = lineXClarity;
    lineY            = lineYClarity;
    xValues          = xValuesClarity;
    yValues          = yValuesClarity;

    itemNumberPrevious = itemNumberPreviousClarity;
    iteratorPrevious   = iteratorPreviousClarity;

    recalculateLine();

    plot->graph(0)->setData(lineX,lineY);
    plot->graph(1)->setData(vectorPointX,vectorPointY);
    plot->replot();
}

void IntensityHistogram::slotBaC()
{
    isBaC = true;
    isClarity = false;

    // Управление графиком передается графиком регулировки яркости и контрастности (BaC)
    qDebug() << "Яркость и контрастность";
    isDeletePoint = true;

    lineXPreviousClarity    = lineXPrevious;
    lineYPreviousClarity    = lineYPrevious;
    vectorPointXClarity     = vectorPointX;
    vectorPointYClarity     = vectorPointY;
    vectorCoordPointClarity = vectorCoordPoint;
    pointsForLineXClarity   = pointsForLineX;
    pointsForLineYClarity   = pointsForLineY;
    lineXClarity            = lineX;
    lineYClarity            = lineY;
    xValuesClarity          = xValues;
    yValuesClarity          = yValues;

    plot->graph(3)->setData(lineX,lineY);
    plot->graph(4)->setData(vectorPointX,vectorPointY);

    itemNumberPreviousClarity = itemNumberPrevious;
    iteratorPreviousClarity   = iteratorPrevious;

    lineXPrevious    = lineXPreviousBaC;
    lineYPrevious    = lineYPreviousBaC;
    vectorPointX     = vectorPointXBaC;
    vectorPointY     = vectorPointYBaC;
    vectorCoordPoint = vectorCoordPointBaC;
    pointsForLineX   = pointsForLineXBaC;
    pointsForLineY   = pointsForLineYBaC;
    lineX            = lineXBaC;
    lineY            = lineYBaC;
    xValues          = xValuesBaC;
    yValues          = yValuesBaC;

    itemNumberPrevious = itemNumberPreviousBaC;
    iteratorPrevious   = iteratorPreviousBaC;

    recalculateLine();

    plot->graph(0)->setData(lineX,lineY);
    plot->graph(1)->setData(vectorPointX,vectorPointY);
    plot->replot();
}

void IntensityHistogram::changeBoundaries(int lower, int upper)
{
    offsetMargin = abs(((plot->xAxis->pixelToCoord(imprecision)) - plot->xAxis->pixelToCoord(imprecision * 2)));
    offsetMarginY = abs(2 * ((plot->yAxis->pixelToCoord(imprecision)) - plot->yAxis->pixelToCoord(imprecision * 2)));

    // Изменение видимых границ графика
    double percentLower = (static_cast<double>(lower)) / 100;
    double percentUpper = (static_cast<double>(upper)) / 100 + 0.01;
    double dif = percentUpper - percentLower;

    minCoordinateX = percentLower * axisLenghtX;
    maxCoordinateX = percentUpper * axisLenghtX;

    plot->xAxis->setRange(minCoordinateX - offsetMargin * dif, maxCoordinateX + dif * offsetMargin);
    recalculationQPoint();
    plot->replot();
}

void IntensityHistogram::slotSmallerSize()
{
    double upperPosition = plot->xAxis->range().upper;
    double lowerPosition = plot->xAxis->range().lower;

    offsetMargin = abs(((plot->xAxis->pixelToCoord(imprecision)) - plot->xAxis->pixelToCoord(imprecision * 2)));
    offsetMarginY = abs(2 * ((plot->yAxis->pixelToCoord(imprecision)) - plot->yAxis->pixelToCoord(imprecision * 2)));

    // Проверка, меньше ли график первоначального вида
    if (plot->xAxis->range().lower <= 0 - offsetMargin ||
        plot->xAxis->range().upper >= axisLenghtX + offsetMargin) {

        plot->xAxis->setRange(0 - offsetMargin, axisLenghtX + offsetMargin);
        plot->yAxis->setRange(-offsetMarginY, maxCoordinateY + offsetMarginY);
        plot->replot();
    }
    recalculationQPoint();
}

void IntensityHistogram::slotWheelToSlider()
{
    // Слот, обрабатывающий взаимосвязь колесика мыши и слайдера
    double upperPosition = plot->xAxis->range().upper;
    double lowerPosition = plot->xAxis->range().lower;

    offsetMargin = abs(((plot->xAxis->pixelToCoord(imprecision)) - plot->xAxis->pixelToCoord(imprecision * 2)));
    offsetMarginY = abs(2 * ((plot->yAxis->pixelToCoord(imprecision)) - plot->yAxis->pixelToCoord(imprecision * 2)));

    if (((upperPosition - lowerPosition) / (INTENSITY_SIZE + 2 * offsetMargin)) * 100
            > 10) {

        lowerPosition = lowerPosition / (INTENSITY_SIZE + 2 * offsetMargin) * 100;
        upperPosition = (upperPosition / (INTENSITY_SIZE + 2 * offsetMargin) * 100) + 1;

        if (upperPosition > 99) {
            upperPosition = 99;
        }

        if (lowerPosition < 0) {
            lowerPosition = 0;
        }

        slider->setLowerPosition(static_cast<int>(lowerPosition));
        slider->setUpperPosition(static_cast<int>(upperPosition));

        recalculationQPoint();
    }
}

void IntensityHistogram::slotMouseWheel(QWheelEvent *event)
{
    double upperPosition = plot->xAxis->range().upper;
    double lowerPosition = plot->xAxis->range().lower;

    // Проверка, находится курсор под графиком
    if (plot->yAxis->pixelToCoord(event->posF().y()) < offsetMarginY)
    {
        isZoom = false;
    }
    else {
        isZoom = true;
    }

    if ((plot->xAxis->range().lower <= 0               ||
            plot->xAxis->range().upper >= axisLenghtX) &&
            !isZoom) {
        offsetMargin = abs(((plot->xAxis->pixelToCoord(imprecision)) - plot->xAxis->pixelToCoord(imprecision * 2)));
        offsetMarginY = abs(2 * ((plot->yAxis->pixelToCoord(imprecision)) - plot->yAxis->pixelToCoord(imprecision * 2)));
        plot->xAxis->setRange(0 - offsetMargin, axisLenghtX + offsetMargin);
        plot->yAxis->setRange(-offsetMarginY, maxCoordinateY + offsetMarginY);
        plot->setInteractions(0);
        slotSmallerSize();
    }
    else {
        plot->setInteractions(QCP::iRangeZoom);
        plot->axisRect()->setRangeZoom(Qt::Horizontal);
    }
}

bool IntensityHistogram::calculateSpline(const std::vector<Point2D> &values, std::vector<Segment> &bezier)
{
    // Пересчет линии граика с помощью кривых Безье
    int n = values.size() - 1;

        if (n < 2)
            return false;

        bezier.resize(n);

        Point2D tgL;
        Point2D tgR;
        Point2D cur;
        Point2D next = values[1] - values[0];
        next.normalize();

        double l1, l2, tmp, x;

        for (int i = 0; i < n; ++i)
        {
            bezier[i].points[0] = bezier[i].points[1] = values[i];
            bezier[i].points[2] = bezier[i].points[3] = values[i + 1];

            tgL = tgR;
            cur = next;

            if(i + 1 < n){
                next = values[i + 2] - values[i + 1];
                next.normalize();

                tgR = cur + next;
                tgR.normalize();
            }else{
                tgR.x = 0.0;
                tgR.y = 0.0;
            }

            if (abs(values[i + 1].y - values[i].y) < EPSILON)
            {
                l1 = l2 = 0.0;
            }
            else
            {
                tmp = values[i + 1].x - values[i].x;
                l1 = abs(tgL.x) > EPSILON ? tmp / (2.0 * tgL.x) : 1.0;
                l2 = abs(tgR.x) > EPSILON ? tmp / (2.0 * tgR.x) : 1.0;
            }

            if (abs(tgL.x) > EPSILON && abs(tgR.x) > EPSILON)
            {
                tmp = tgL.y / tgL.x - tgR.y / tgR.x;
                if (abs(tmp) > EPSILON)
                {
                    x = (values[i + 1].y - tgR.y / tgR.x * values[i + 1].x - values[i].y + tgL.y / tgL.x * values[i].x) / tmp;
                    if (x > values[i].x && x < values[i + 1].x)
                    {
                        if (tgL.y > 0.0)
                        {
                            if (l1 > l2)
                                l1 = 0.0;
                            else
                                l2 = 0.0;
                        }
                        else
                        {
                            if (l1 < l2)
                                l1 = 0.0;
                            else
                                l2 = 0.0;
                        }
                    }
                }
            }

            bezier[i].points[1] += tgL * l1;
            bezier[i].points[2] -= tgR * l2;
        }

        return true;
}

void IntensityHistogram::setTracer()
{
    // Работа с трассировщиком
    verticalLine = new QCPCurve(plot->xAxis, plot->yAxis);
    verticalLine->setPen(Qt::NoPen);

#if CONTINIOUSLY
    tracer = new QCPItemTracer(plot);
    tracer->setGraph(plot->graph(0));
    tracer->setPen(Qt::NoPen);
#endif
}

void IntensityHistogram::setDefaultGraphSettings()
{
    // Начальные настройки графика
    std::vector<Point2D> testValues;
    std::vector<Segment> spline;
    Point2D p;

    pointsForLineX.push_back(0);
    pointsForLineX.push_back(65535);
    pointsForLineX.push_back(65536);

    pointsForLineY.push_back(0);
    pointsForLineY.push_back(65535);
    pointsForLineY.push_back(65535.001);

    testValues.push_back(Point2D(0, 0));
    testValues.push_back(Point2D(65535, 65535));
    testValues.push_back(Point2D(65536, 65536));

    vectorPointX.push_back(0);
    vectorPointX.push_back(65535);

    QPointF firstPoint(plot->xAxis->coordToPixel(0),plot->yAxis->coordToPixel(0));
    QPointF secondPoint(plot->xAxis->coordToPixel(65535),plot->yAxis->coordToPixel(65535));
    vectorCoordPoint.push_back(firstPoint);
    vectorCoordPoint.push_back(secondPoint);

    vectorPointY.push_back(0);
    vectorPointY.push_back(65535);

    xValues.resize(65536);
    yValues.resize(65536);

    calculateSpline(testValues, spline);

    for (auto s : spline)
    {
        for (int i = 0; i < RESOLUTION; ++i)
        {
            s.calc((double)i / (double)RESOLUTION, p);
            lineX.push_back(p.x);
            lineY.push_back(p.y);
        }
    }

    plot->clearGraphs();
    plot->addGraph();
    plot->xAxis->setRange(0 - offsetMargin, 65536 + offsetMargin);
    plot->yAxis->setRange(-offsetMarginY, maxCoordinateY + offsetMarginY);
    plot->graph(0)->setData(lineX,lineY);

    // Удаление подписей осей
    plot->xAxis->setTicks(false);
    plot->yAxis->setTicks(false);
    plot->yAxis->setVisible(false);

    // Удаление отступов от осей-
    plot->axisRect()->setAutoMargins(QCP::msNone);

    // Удаление сетки
    plot->xAxis->grid()->setVisible(false);
    plot->yAxis->grid()->setVisible(false);

    // Добавление графика точек
    addPointGraph();

    // График гистограммы
    plot->addGraph();
    plot->graph(2)->setPen(QPen(Qt::darkCyan));
    plot->graph(2)->setLineStyle(QCPGraph::lsImpulse);

    // Добавление второй линии
    plot->addGraph();
    plot->graph(3)->setData(lineX,lineY);
    plot->graph(3)->setPen(QPen(Qt::gray));

    plot->addGraph();
    plot->graph(4)->setScatterStyle(QCPScatterStyle::ssCircle);
    plot->graph(4)->setLineStyle(QCPGraph::lsNone);
    plot->graph(4)->setPen(QPen(QBrush(Qt::gray),imprecision));

    plot->addLayer("0");
    plot->addLayer("1");
    plot->addLayer("2");
    plot->addLayer("3");
    plot->addLayer("4");

    plot->graph(2)->setLayer("0");
    plot->graph(0)->setLayer("3");
    plot->graph(1)->setLayer("4");
    plot->graph(3)->setLayer("1");
    plot->graph(4)->setLayer("2");

    // Масштабирование графика
    plot->setInteractions(QCP::iRangeZoom);
    plot->axisRect()->setRangeZoom(Qt::Horizontal);

    setTracer();

#if CONTINIOUSLY
    plot->graph(1)->setVisible(true);
    plot->graph(0)->setVisible(true);
#endif

    plot->graph(1)->setData(vectorPointX,vectorPointY);
    plot->graph(4)->setData(vectorPointX,vectorPointY);
    setDefaultSettingsForLines();
    plot->replot();
}

void IntensityHistogram::setDefaultSettingsForLines()
{
    // Установка начальных значений в линии яркости и яркости с контрастностью (BaC)
    lineXPreviousClarity    = lineXPrevious;
    lineYPreviousClarity    = lineYPrevious;
    vectorPointXClarity     = vectorPointX;
    vectorPointYClarity     = vectorPointY;
    vectorCoordPointClarity = vectorCoordPoint;
    pointsForLineXClarity   = pointsForLineX;
    pointsForLineYClarity   = pointsForLineY;
    lineXClarity            = lineX;
    lineYClarity            = lineY;
    xValuesClarity          = xValues;
    yValuesClarity          = yValues;

    lineXPreviousBaC        = lineXPrevious;
    lineYPreviousBaC        = lineYPrevious;
    vectorPointXBaC         = vectorPointX;
    vectorPointYBaC         = vectorPointY;
    vectorCoordPointBaC     = vectorCoordPoint;
    pointsForLineXBaC       = pointsForLineX;
    pointsForLineYBaC       = pointsForLineY;
    lineXBaC                = lineX;
    lineYBaC                = lineY;
    xValuesBaC              = xValues;
    yValuesBaC              = yValues;
}

void IntensityHistogram::addSlider()
{
    // Добавление слайдера с начальными параметрами
    slider = new QxtSpanSlider;
    slider->setOrientation(Qt::Orientation::Horizontal);

    slider->setLowerValue(0);
    slider->setUpperValue(100);

    slider->setLowerPosition(0);
    slider->setUpperPosition(99);

    slider->setHandleMovementMode(QxtSpanSlider::HandleMovementMode::NoOverlapping);
    connect(slider,SIGNAL(spanChanged(int,int)),this,SLOT(changeBoundaries(int,int)));
}

void IntensityHistogram::addInversionButton()
{
    // Добавление кнопки инверсии
    inversionButton = new QPushButton();
    inversionButton->setText("Инверсия");
    connect(inversionButton,SIGNAL(clicked()),this,SLOT(slotInversion()));
}

void IntensityHistogram::addClarityButton()
{
    // Добавление кнопки управления прозрачностью
    clarityButton = new QPushButton();
    clarityButton->setText("Прозрачность");
    connect(clarityButton,SIGNAL(clicked()),this,SLOT(slotClarity()));
}

void IntensityHistogram::addBaCButton()
{
    // Добавление кнопки управления яркостью и контрастностью (BaC)
    bacButton = new QPushButton();
    bacButton->setText("Яркость и контрастность");
    connect(bacButton,SIGNAL(clicked()),this,SLOT(slotBaC()));
}

void IntensityHistogram::savePreviousState()
{
    // СОхранение предыдущего состояния графика, необходимо для изменения положения линии
    lineXPrevious = lineX;
    lineYPrevious = lineY;

    int itemNumber = itemSearch();
    coordXPrevious = lineX[itemNumber];
    coordYPrevious = lineY[itemNumber];
}

void IntensityHistogram::changeLine(double coordX, double coordY)
{
    // Изменение текущего положения точки в линии
    int itemNumber = itemSearch();

    if (itemNumberPrevious > lineX.size()) {
        findItemNumberPrevious();
    }

    if (itemNumber != itemNumberPrevious) {
        lineX[itemNumber] = coordX;
        lineY[itemNumber] = coordY;
        lineX[itemNumberPrevious] = coordXPrevious;
        lineY[itemNumberPrevious] = coordYPrevious;
    }

    else {
        lineX[itemNumber] = coordX;
        lineY[itemNumber] = coordY;
    }
    itemNumberPrevious = itemNumber;
}

void IntensityHistogram::redrawGraph(QMouseEvent *event)
{
    // Перерисовка графика
    double coordX = plot->xAxis->pixelToCoord(event->pos().x());
    double coordY = plot->yAxis->pixelToCoord(event->pos().y());

    int height = plot->height();

    double pointsToHeight = height / imprecision;
    double pointDeviation = (percentForDeviation / 100) * (static_cast<double>(imprecision));

    std::vector<Point2D> testValues;
    std::vector<Segment> spline;
    Point2D p;

    bool isBoundaryPoint = false;
    bool isDeleteBoundaryPoint = true;

    // Блок обработки захвата крайних точек
    for (int i = 0; i < 2; i++)
    {
        if ((abs(plot->xAxis->coordToPixel(coordX) -
                vectorCoordPoint.at(i).x()) <= imprecision * 2 /*&&
                abs(plot->yAxis->coordToPixel(coordY) -
                vectorCoordPoint.at(i).y()) <= imprecision * 2*/) &&
                !isNonBoudaryPoint) {

                double x = vectorPointX.at(i);
                QPointF point(vectorCoordPoint.at(i).x(), plot->yAxis->coordToPixel(coordY));
                vectorPointY[i] = coordY;
                vectorCoordPoint[i] = point;

                for (int j = 0; j < pointsForLineX.size() - 1; j++)
                {
                    if (pointsForLineX.at(j) == x) {
                        pointsForLineY[j] = coordY;
                        isPointCaptured = true;
                        isBoundaryPoint = true;
                        isDeletePoint = false;
                        isDeleteBoundaryPoint = false;
                        isBoundaryPointCap = true;
                        isThreePoints();
                    }
                }
        }
    }

    // Блок обработки захвата поставленной точки
    for (int i = 2; i < vectorPointX.size(); i++)
    {
        if (abs(plot->xAxis->coordToPixel(coordX) -
                vectorCoordPoint.at(i).x()) < (imprecision + pointDeviation) &&
                abs(plot->yAxis->coordToPixel(coordY) -
                vectorCoordPoint.at(i).y()) < (imprecision * pointsToHeight) &&
                !isPointCaptured) {
                double x = vectorPointX.at(i);
                vectorPointX.remove(i);
                vectorPointY.remove(i);
                vectorCoordPoint.remove(i);

                for (int j = 0; j < pointsForLineX.size() - 1; j++)
                {
                    if (pointsForLineX.at(j) == x) {
                        pointsForLineX.removeAt(j);
                        pointsForLineY.removeAt(j);
                        itemNumberPrevious--;
                        isNonBoudaryPoint = true;
                    }
                }
        }
    }

    if (graphChangeState &&
            !isBoundaryPoint &&
            isDeleteBoundaryPoint &&
            isDeletePoint &&
            !isBoundaryPointCap) {
        pointsForLineX.removeAt(iteratorPrevious);
        pointsForLineY.removeAt(iteratorPrevious);
        clearPoint();
    }

    if (isDeleteBoundaryPoint) {
        isDeletePoint = true;
    }

    isPointCaptured = true;
    xCrossesPoint(coordX, coordY);

    for (int i = 0; i < pointsForLineX.size() - 1; i++)
    {
        if (coordX > (pointsForLineX.at(i)) &&
                coordX < (pointsForLineX.at(i + 1)) && !isBoundaryPointCap) {
            pointsForLineX.insert(i + 1, coordX);
            pointsForLineY.insert(i + 1, coordY);
            iteratorPrevious = i + 1;
            addPoint(coordX, coordY);
        }
    }

    for (int i = 2; i < vectorPointX.size(); i++)
    {
        if ((((vectorCoordPoint.at(i).x() < (2 * imprecision)) &&
                (abs(vectorCoordPoint.at(i).y() - vectorCoordPoint.at(0).y()) < (2 * imprecision))) ||
                ((vectorCoordPoint.at(i).x() > vectorCoordPoint.at(1).x() - (2 * imprecision)) &&
                (abs(vectorCoordPoint.at(i).y() - vectorCoordPoint.at(1).y()) < (2 * imprecision)))) &&
                isPointCaptured && isBoundaryPoint) {

            double x = vectorPointX.at(i);
            vectorPointX.remove(i);
            vectorPointY.remove(i);
            vectorCoordPoint.remove(i);

            for (int j = 0; j < pointsForLineX.size() - 1; j++)
            {
                if (pointsForLineX.at(j) == x) {
                    pointsForLineX.removeAt(j);
                    pointsForLineY.removeAt(j);
                    recalculationQPoint();
                }
            }
        }
        isThreePoints();
    }

    graphChangeState = true;
    isPointsGoBeyond();

    for (int i = 0; i < pointsForLineX.size() - 1; i++)
    {
        testValues.push_back(Point2D(pointsForLineX[i], pointsForLineY[i]));
    }

    calculateSpline(testValues, spline);
    QVector<double> lineXChanged, lineYChanged;

    for (auto s : spline)
    {
        for (int i = 0; i < RESOLUTION; ++i)
        {
            s.calc((double)i / (double)RESOLUTION, p);
            lineXChanged.push_back(p.x);
            lineYChanged.push_back(p.y);
        }
    }

    lineX = lineXChanged;
    lineY = lineYChanged;

    isLineGoBeyond();
    recalculationQPoint();

    plot->graph(0)->setData(lineX,lineY);
    plot->graph(1)->setData(vectorPointX,vectorPointY);
}

void IntensityHistogram::xCrossesPoint(double &coordX, double &coordY)
{
    // Проверка, есть ли пересечение захваченной точки и существующих
    if (!intersectionState) {
        for (int i = 0; i < pointsForLineX.size() - 2; i++)
        {
            if ((coordX > pointsForLineX.at(i) &&
                    coordX < pointsForLineX.at(i + 1))) {
                currentPointX = i;
                nextPointX = i + 1;
            }
        }
        intersectionState = true;
    }

    bool isBoundaryPoint = false;

    double pointDeviation = (percentForDeviation / 100) * (static_cast<double>(imprecision));
    double deviationX = abs(plot->xAxis->pixelToCoord(0) -
            plot->xAxis->pixelToCoord(2 * imprecision + pointDeviation));

    if (coordX <= pointsForLineX.at(currentPointX) + deviationX && !isBoundaryPoint) {
        coordX = pointsForLineX.at(currentPointX) + deviationX;
    }

    if (coordX >= pointsForLineX.at(nextPointX) - deviationX && !isBoundaryPoint) {
        coordX = pointsForLineX.at(nextPointX) - deviationX;
    }
}

void IntensityHistogram::addPointGraph()
{
    // Добавление графика точек
    plot->addGraph();
    plot->graph(1)->setScatterStyle(QCPScatterStyle::ssCircle);
    plot->graph(1)->setLineStyle(QCPGraph::lsNone);
    plot->graph(1)->setPen(QPen(QBrush(Qt::red),imprecision));
}

void IntensityHistogram::addPoint(double x, double y)
{
    // Добавление точки в вектор точек
    vectorPointX.append(x);
    vectorPointY.append(y);
    QPointF point(plot->xAxis->coordToPixel(x),plot->yAxis->coordToPixel(y));
    vectorCoordPoint.push_back(point);
    plot->graph(1)->setData(vectorPointX, vectorPointY);
}

void IntensityHistogram::clearPoint()
{
    // Удалить точку из вектора точек
    vectorPointX.pop_back();
    vectorPointY.pop_back();
    vectorCoordPoint.pop_back();
}

void IntensityHistogram::findItemNumberPrevious()
{
    // При удалении точки можетпотребоваться пересчет itemNumberPrevious
    itemNumberPrevious = itemSearch();
}

void IntensityHistogram::recalculationQPoint()
{
    // Пересчет точек, содержащих данные об метках на графике
    for (int i = 0; i < vectorCoordPoint.size(); i++)
    {
       double x = plot->xAxis->coordToPixel(vectorPointX[i]);
       double y = plot->yAxis->coordToPixel(vectorPointY[i]);
       QPointF point(x,y);
       vectorCoordPoint[i] = point;
    }
}

void IntensityHistogram::recalculateLine()
{
    // Пересчет линии кривыми Безье
    isThreePoints();
    recalculationQPoint();
    isLineGoBeyond();

    std::vector<Point2D> testValues;
    std::vector<Segment> spline;
    Point2D p;

    for (int i = 0; i < pointsForLineX.size() - 1; i++)
    {
        testValues.push_back(Point2D(pointsForLineX[i], pointsForLineY[i]));
    }

    calculateSpline(testValues, spline);

    QVector<double> lineXChanged, lineYChanged;

    for (auto s : spline)
    {
        for (int i = 0; i < RESOLUTION; ++i)
        {
            s.calc((double)i / (double)RESOLUTION, p);
            lineXChanged.push_back(p.x);
            lineYChanged.push_back(p.y);
        }
    }

    lineX = lineXChanged;
    lineY = lineYChanged;

    isPointsGoBeyond();
}

void IntensityHistogram::clearData()
{
    // Очистка данных
    pointsForLineX.clear();
    pointsForLineY.clear();
    vectorPointX.clear();
    vectorPointY.clear();
    vectorCoordPoint.clear();
    xValues.clear();
    yValues.clear();
    lineXPrevious.clear();
    lineYPrevious.clear();
    lineX.clear();
    lineY.clear();
}

void IntensityHistogram::clearLine()
{
    offsetMargin = abs(((plot->xAxis->pixelToCoord(imprecision)) - plot->xAxis->pixelToCoord(imprecision * 2)));
    offsetMarginY = abs(2 * ((plot->yAxis->pixelToCoord(imprecision)) - plot->yAxis->pixelToCoord(imprecision * 2)));

    std::vector<Point2D> testValues;
    std::vector<Segment> spline;
    Point2D p;

    clearData();

    pointsForLineX.push_back(0);
    pointsForLineX.push_back(65535);
    pointsForLineX.push_back(65536);

    pointsForLineY.push_back(0);
    pointsForLineY.push_back(65535);
    pointsForLineY.push_back(65535.001);

    testValues.push_back(Point2D(0, 0));
    testValues.push_back(Point2D(65535, 65535));
    testValues.push_back(Point2D(65536, 65536));

    vectorPointX.push_back(0);
    vectorPointX.push_back(65535);

    QPointF firstPoint(plot->xAxis->coordToPixel(0),plot->yAxis->coordToPixel(0));
    QPointF secondPoint(plot->xAxis->coordToPixel(65535),plot->yAxis->coordToPixel(65535));
    vectorCoordPoint.push_back(firstPoint);
    vectorCoordPoint.push_back(secondPoint);

    vectorPointY.push_back(0);
    vectorPointY.push_back(65535);

    xValues.resize(65536);
    yValues.resize(65536);

    slider->setLowerPosition(static_cast<int>(0));
    slider->setUpperPosition(static_cast<int>(99));

    calculateSpline(testValues, spline);

    for (auto s : spline)
    {
        for (int i = 0; i < RESOLUTION; ++i)
        {
            s.calc((double)i / (double)RESOLUTION, p);
            lineX.push_back(p.x);
            lineY.push_back(p.y);
        }
    }

    plot->xAxis->setRange(-offsetMargin, 65536 + offsetMargin);
    plot->yAxis->setRange(-offsetMarginY, maxCoordinateY + offsetMarginY);

    recalculationQPoint();

    plot->graph(0)->setData(lineX,lineY);
    plot->graph(1)->setData(vectorPointX,vectorPointY);
}

void IntensityHistogram::doAutoContrast(QVector<double>& _vectorY, double _maxY)
{
    if (isAutoContrast) {
        bool isFirstValueFound = false;
        bool isLastValueFound = false;
        double maxY = _maxY;
        double startOfAutoContrast = 0;
        double endOfAutoContrast = 0;
        QVector<double> yValue = _vectorY;

        for (int i = 1; i < yValue.size(); i++)
        {
            if ((yValue.at(i) > (percentAutoContrast * INTENSITY_MAX)) &&
                    isLastValueFound)  {
                isLastValueFound = false;
            }
            if ((yValue.at(i) > (percentAutoContrast * INTENSITY_MAX)) &&
                !isFirstValueFound) {
                    isFirstValueFound = true;
                    startOfAutoContrast = i;
            }
            if ((yValue.at(i) < (percentAutoContrast * INTENSITY_MAX)) &&
                    isFirstValueFound && !isLastValueFound) {
                isLastValueFound = true;
                endOfAutoContrast = i;
            }
        }

        if (isFirstValueFound && isLastValueFound) {
            clearLine();
            lowerValueForBoundaries = static_cast<int>((startOfAutoContrast / INTENSITY_MAX) * 100);
            upperValueForBoundaries = static_cast<int>((endOfAutoContrast / INTENSITY_MAX) * 100);

            slider->setLowerValue(lowerValueForBoundaries);
            slider->setUpperValue(upperValueForBoundaries);

            double minCorrdY = 0;
            double maxCoordY = INTENSITY_MAX;

            double pointDeviation = (percentForDeviation / 100) * (static_cast<double>(imprecision));

            for (int i = 2; i < vectorPointX.size(); i++)
            {
                if ((abs(plot->xAxis->coordToPixel(startOfAutoContrast) -
                        vectorCoordPoint.at(i).x()) <= (imprecision + pointDeviation))) {
                    double x = vectorPointX.at(i);
                    vectorPointX.remove(i);
                    vectorPointY.remove(i);
                    vectorCoordPoint.remove(i);

                    for (int j = 0; j < pointsForLineX.size() - 1; j++)
                    {
                        if (pointsForLineX.at(j) == x) {
                            pointsForLineX.removeAt(j);
                            pointsForLineY.removeAt(j);
                            //itemNumberPrevious--;
                        }
                    }
                }
                if ((abs(plot->xAxis->coordToPixel(endOfAutoContrast) -
                        vectorCoordPoint.at(i).x()) <= (imprecision + pointDeviation))) {
                    double x = vectorPointX.at(i);
                    vectorPointX.remove(i);
                    vectorPointY.remove(i);
                    vectorCoordPoint.remove(i);

                    for (int j = 0; j < pointsForLineX.size() - 1; j++)
                    {
                        if (pointsForLineX.at(j) == x) {
                            pointsForLineX.removeAt(j);
                            pointsForLineY.removeAt(j);
                            //itemNumberPrevious--;
                        }
                    }
                }
            }

            for (int i = 0; i < pointsForLineX.size() - 1; i++)
            {
                if (startOfAutoContrast > (pointsForLineX.at(i)) &&
                        startOfAutoContrast < (pointsForLineX.at(i + 1)) &&
                        (abs(plot->xAxis->coordToPixel(startOfAutoContrast) -
                        pointsForLineX.at(i)) > (imprecision + pointDeviation))) {
                    pointsForLineX.insert(i + 1, startOfAutoContrast);
                    pointsForLineY.insert(i + 1, minCorrdY);
                    iteratorPrevious = i + 1;
                    addPoint(startOfAutoContrast, 0);
                }
                if (endOfAutoContrast > (pointsForLineX.at(i)) &&
                        endOfAutoContrast < (pointsForLineX.at(i + 1)) &&
                        (abs(plot->xAxis->coordToPixel(endOfAutoContrast) -
                        pointsForLineX.at(i)) > (imprecision + pointDeviation))) {
                    pointsForLineX.insert(i + 1, endOfAutoContrast);
                    pointsForLineY.insert(i + 1, maxCoordY);
                    iteratorPrevious = i + 1;
                    addPoint(endOfAutoContrast, INTENSITY_MAX);
                }
            }
            isThreePoints();
        }
    }
}

void IntensityHistogram::setXYValues()
{
    // Рассчитать все значения по Х и У в вектор значений с шагом в 1
    int j = 0;
    lineY.push_back(plot->yAxis->pixelToCoord(vectorCoordPoint.at(1).y()));
    lineX.push_back(plot->xAxis->pixelToCoord(vectorCoordPoint.at(1).x()));
    for (int i = 0; i < axisLenghtX + 1; i++)
    {

        if (static_cast<int>(lineX[j]) == i) {
            yValues[i] = lineY.at(j);
            j++;
        }

        if (i > static_cast<int>(lineX.at(j - 1)) &&
                i < static_cast<int>(lineX.at(j))) {
            double k = (i - lineX.at(j - 1)) / (lineX.at(j) - lineX.at(j - 1));
            yValues[i] = lineY.at(j - 1) + ((lineY.at(j) - lineY.at(j - 1)) * k);
        }
    }

    lineY.pop_back();
    lineX.pop_back();
}

int IntensityHistogram::itemSearch()
{
    // Поиск и возврат текущего элемента в массиве
    for (int i = 0; i < lineX.size() - 1; i++)
    {
        if (lineX[i] == tracer->position->key()) {
            return i;
        }
    }
    return 0;
}

bool IntensityHistogram::isThreePoints()
{
    // Проверка, хватает ли точек для построения прямой.
    if (vectorCoordPoint.size() == 2) {
        double x = (minCoordinateX + maxCoordinateX) / 2;
        double y = (minCoordinateY + maxCoordinateY) / 2;
        addPoint(x,y);
        pointsForLineX.insert(1,x);
        pointsForLineY.insert(1,y);
        itemNumberPrevious++;
        return true;
    }
}

bool IntensityHistogram::isLineGoBeyond()
{
    // Проверка, вышла ли линия за пределы графика
    for (int i = 0; i < lineY.size(); i++)
    {
        if (lineY.at(i) > maxCoordinateY * 1.0) {
            lineY[i] = maxCoordinateY * 1.0;
        }
        if (lineY.at(i) < 0) {
            lineY[i] = 0;
        }
    }
    return true;
}

bool IntensityHistogram::isPointsGoBeyond()
{
    // Проверка, вышла ли точка за пределы графика
    for (int i = 0; i < vectorPointY.size(); i++)
    {
        if (vectorPointY.at(i) > maxCoordinateY * 1.0) {
            vectorPointY[i] = maxCoordinateY * 1.0;
        }
        if (vectorPointY.at(i) < 0) {
            vectorPointY[i] = 0;
        }
    }

    for (int i = 0; i < pointsForLineY.size(); i++)
    {
        if (pointsForLineY.at(i) > maxCoordinateY * 1.0) {
            pointsForLineY[i] = maxCoordinateY * 1.0;
        }
        if (pointsForLineY.at(i) < 0) {
            pointsForLineY[i] = 0;
        }
    }
    return true;
}

void IntensityHistogram::on_pushButton_clicked()
{
    offsetMargin = abs(((plot->xAxis->pixelToCoord(imprecision)) - plot->xAxis->pixelToCoord(imprecision * 2)));
    offsetMarginY = abs(2 * ((plot->yAxis->pixelToCoord(imprecision)) - plot->yAxis->pixelToCoord(imprecision * 2)));

    ImageAcquisition acquisition;
    // Добавляем один график в widget
    plot->addGraph();
    // Говорим, что отрисовать нужно график по нашим двум массивам x и y
    plot->graph(2)->setData(acquisition.getVectorX(), acquisition.getVectorY());
    plot->graph(2)->setPen(QPen(Qt::darkCyan));

    plot->addLayer("0");
    plot->addLayer("1");
    plot->addLayer("2");

    plot->graph(2)->setLayer("0");
    plot->graph(0)->setLayer("1");
    plot->graph(1)->setLayer("2");

    maxCoordinateY = acquisition.getMaxY() * (INTENSITY_MAX / acquisition.getMaxY());
    minCoordinateY = acquisition.getMinY();

    pointsForLineY[1] = maxCoordinateY;
    pointsForLineY[2] = maxCoordinateY + 0.001;
    vectorPointY[1]   = maxCoordinateY;

    plot->yAxis->setRange(minCoordinateY, maxCoordinateY + offsetMarginY);
    changeBoundaries(slider->lowerValue(), slider->upperValue());
}

void IntensityHistogram::clarityIsInvisible()
{
    // Сделать невидимой линию прозрачности
    plot->graph(3)->setVisible(false);
    plot->graph(4)->setVisible(false);
    plot->replot();
}

void IntensityHistogram::clearPlot()
{
    plot->removeGraph(0);
    plot->removeGraph(1);
    plot->removeGraph(2);
    plot->removeGraph(3);
    plot->removeGraph(4);

    clearData();

    setDefaultGraphSettings();
}

bool IntensityHistogram::getBaCFlag()
{
    return isBaC;
}

bool IntensityHistogram::getClarityFlag()
{
    return isClarity;
}

void IntensityHistogram::setAutoContrast(bool flagAutoContrast)
{
    isAutoContrast = flagAutoContrast;
}

bool IntensityHistogram::getAutoContrast()
{
    return isAutoContrast;
}
