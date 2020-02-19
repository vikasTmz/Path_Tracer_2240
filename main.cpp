#include <QCoreApplication>
#include <QCommandLineParser>

#include <iostream>

#include "pathtracer.h"
#include "scene/scene.h"

#include <QImage>

#include "util/CS123Common.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addPositionalArgument("scene", "Scene file to be rendered");
    parser.addPositionalArgument("output", "Image file to write the rendered image to");
    parser.addPositionalArgument("num_samples", "Number of Samples");

    parser.process(a);

    const QStringList args = parser.positionalArguments();
    if(args.size() != 3) {
        std::cerr << "Error: Wrong number of arguments" << std::endl;
        std::cerr << "Usage: Refer to README" << std::endl;
        a.exit(1);
        return 1;
    }
    bool ok;

    QString scenefile = args[0];
    QString output = args[1];
    quint16 num_samples = 0;
    num_samples = args[2].toUShort(&ok, 10); // unsigned short int as number of samples will realistically never be large.

    QImage image(IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB32);

    Scene *scene;
    if(!Scene::load(scenefile, &scene)) {
        std::cerr << "Error parsing scene file " << scenefile.toStdString() << std::endl;
        a.exit(1);
        return 1;
    }

    PathTracer tracer(IMAGE_WIDTH, IMAGE_HEIGHT, num_samples);

    QRgb *data = reinterpret_cast<QRgb *>(image.bits());

    QTime myTimer;
    myTimer.start();

    tracer.traceScene(data, *scene);
    delete scene;

    bool success = image.save(output);
    if(!success) {
        success = image.save(output, "PNG");
    }
    if(success) {
        std::cout << "Wrote rendered image to " << output.toStdString() << std::endl;
        float nMilliseconds = myTimer.elapsed();
        std::cout << "Total time to render (seconds) = " << nMilliseconds / 1000.0f << std::endl;
        std::cout << "Total time to render (minutes) = " << nMilliseconds / 60000.0f << std::endl;
    } else {
        std::cerr << "Error: failed to write image to " << output.toStdString() << std::endl;
    }
    a.exit();
}
