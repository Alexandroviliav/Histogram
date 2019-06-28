#include "mytiff.h"

//myTIFF::myTIFF(QObject *parent) : QObject(parent)
myTIFF::myTIFF()
{
    QLibrary lib2("libTIFF64.dll");
    if(lib2.load())
        qDebug() << "myTIFF::::Библиотека загружена";
    else
        qDebug() << "myTIFF::::Библиотека не загружена";

    myTIFFOpen = (TIFFOpen) QLibrary::resolve("libTIFF64.dll", "TIFFOpen");
    myTIFFGetField = (TIFFGetField) QLibrary::resolve("libTIFF64.dll", "TIFFGetField");
    myTIFFReadScanline = (TIFFReadScanline) QLibrary::resolve("libTIFF64.dll", "TIFFReadScanline");
    myTIFFWriteScanline = (TIFFWriteScanline) QLibrary::resolve("libTIFF64.dll", "TIFFWriteScanline");
    my_TIFFmalloc = (_TIFFmalloc) QLibrary::resolve("libTIFF64.dll", "_TIFFmalloc");
    myTIFFScanlineSize = (TIFFScanlineSize) QLibrary::resolve("libTIFF64.dll", "TIFFScanlineSize");
    myTIFFDefaultStripSize = (TIFFDefaultStripSize) QLibrary::resolve("libTIFF64.dll", "TIFFDefaultStripSize");
    myTIFFSetField = (TIFFSetField) QLibrary::resolve("libTIFF64.dll", "TIFFSetField");
    myTIFFClose = (TIFFClose) QLibrary::resolve("libTIFF64.dll", "TIFFClose");
    myTIFFSetupStrips = (TIFFClose) QLibrary::resolve("libTIFF64.dll", "TIFFSetupStrips");
    data = new ushort[4000*4000];
}

ushort * myTIFF::openFile(const char * path, uint * width, uint * height)
{
TIFF * img = myTIFFOpen(path, "r");
if (img)
{
    uint32 w, h;

    myTIFFGetField(img, TIFFTAG_IMAGEWIDTH, &w);
    myTIFFGetField(img, TIFFTAG_IMAGELENGTH, &h);

    uint32 imagelength;
    tdata_t buf;
    uint32 row;
    delete[] data;
    data = new ushort[w*h];
    memcpy(width, (uint *)&w, SIZEOF_INT);
    memcpy(height, (uint *)&h, SIZEOF_INT);
    tsize_t size = myTIFFScanlineSize(img);
    buf = my_TIFFmalloc(size);
    myTIFFGetField(img, TIFFTAG_IMAGELENGTH, &imagelength);

    for (int i=0;i<h;i++)
    {
        row = i;
        myTIFFReadScanline(img, buf, row, 0);
        ushort * pix = (ushort*) buf;

        memcpy((data+i*w),pix,size);
    }
    return data;
}
else
{
    qDebug() << "myTIFF::ERROR!" << path;
    return NULL;
}
}

void myTIFF::writeFile(const char * path, ushort * data, uint  width, uint  height)
{
qDebug() << "path" << path << width << height;
TIFF *out = myTIFFOpen(path, "w");
if (out)
{
    qDebug() << "inm";
    uint32 imagelength, imagewidth;
    uint16 nsamples =  imagewidth*imagelength;
    imagewidth = width;
    imagelength = height;

    myTIFFSetField(out, TIFFTAG_IMAGELENGTH, imagelength);
    myTIFFSetField(out, TIFFTAG_IMAGEWIDTH,  imagewidth);
    myTIFFSetField(out, TIFFTAG_SAMPLESPERPIXEL, 1);
    myTIFFSetField(out, TIFFTAG_PHOTOMETRIC, 1);
    myTIFFSetField(out, TIFFTAG_COMPRESSION, 1);
    myTIFFSetField(out, TIFFTAG_BITSPERSAMPLE, 16);
    myTIFFSetField(out, TIFFTAG_ROWSPERSTRIP, myTIFFDefaultStripSize(out, imagewidth*imagelength));
    myTIFFSetupStrips(out);

    for (int i = 0; i < height; i++)
    {
        myTIFFWriteScanline(out, &data[i*width], i, 0);
    }

    myTIFFClose(out);
}
}

uint myTIFF::getWidth(const char *path)
{
    TIFF * img = myTIFFOpen(path, "r");
    uint w;
    if (img)
    {
        myTIFFGetField(img, TIFFTAG_IMAGEWIDTH, &w);
    }
    return w;
}

uint myTIFF::getHeight(const char *path)
{
    TIFF * img = myTIFFOpen(path, "r");
    uint h;
    if (img)
    {
        myTIFFGetField(img, TIFFTAG_IMAGELENGTH, &h);
    }
    return h;
}
