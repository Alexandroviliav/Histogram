#include "imageacquisition.h"
#include <QTime>
#include <QDebug>

ImageAcquisition::ImageAcquisition(QObject *parent) : QObject(parent)
{

}

QVector<double> ImageAcquisition::getVectorX()
{
    QVector<double> x(INTENSITY_SIZE);
    for (int i = 0; i < INTENSITY_SIZE; i++)
    {
        x[i] = i;
    }
    return x;
}

QVector<double> ImageAcquisition::getVectorY()
{
//    QVector<double> y2(INTENSITY_SIZE);
    double maxY = sqrt(getMaxY());
    double k = INTENSITY_MAX / maxY;

    for (int i = 0; i < INTENSITY_SIZE; i++)
    {
        y[i] = sqrt(y.at(i)) * k;
    }
    return y;
}

double ImageAcquisition::getMinY()
{
    return minY;
}

double ImageAcquisition::getMaxY()
{
    return maxY;
}

void ImageAcquisition::setParameters(int &width, int &height, ushort *data)
{
    readImageContinuously(width, height, data);
}

void ImageAcquisition::readImageFromFile()
{
    tif = new myTIFF;
//    map = new QMap<int,int>;

    tifWidth = tif->getWidth(imageName);
    tifHeight = tif->getHeight(imageName);
    qDebug() << "tifWidth" << tifWidth;
    qDebug() << "tifHeight" << tifHeight;

    ushort *data = tif->openFile(imageName, &tifWidth, &tifHeight);
    int size = tifWidth * tifHeight;

    QVector<double> x(INTENSITY_SIZE);

    y = x;


    for (int i = 0; i < size; i++)
    {
//        ushort key = data[i];
//        ushort quantity = map->value(key);
//        quantity++;
//        map->insert(key,quantity);
        y[data[i]]++;
    }


//    for (int i = 0; i < INTENSITY_SIZE; i++)
//    {
//        x[i] = i;
//        y[i] = map->value(i);
//    }
    setMinMaxY(y);
}

void ImageAcquisition::readImageContinuously(int &width, int &height, ushort *data)
{

    //map = new QMap<int,int>;
    tifWidth = width;
    tifHeight = height;

    QVector<double> x(INTENSITY_SIZE);

    y = x;

    int size = tifWidth * tifHeight;
    for (int i = 0; i < size; i++)
    {
//        int key = data[i];
//        int quantity = map->value(key);
//        quantity++;
//        map->insert(key,quantity);
        y[data[i]]++;
    }


//    int k = 0;
//    for (int i = 0; i < INTENSITY_SIZE; i++)
//    {
//        //x[i] = i;
//        y[i] = map->value(i);
//    }

    setMinMaxY(y);
}

void ImageAcquisition::setMinMaxY(QVector<double> &_y)
{
    for (int i=1; i < INTENSITY_SIZE; i++)
    {
        if (_y[i] < minY) minY = _y[i];
        if (_y[i] > maxY) maxY = _y[i];
    }
}

int ImageAcquisition::getWidth()
{
    return tifWidth;
}

int ImageAcquisition::getHeight()
{
    return tifHeight;
}
