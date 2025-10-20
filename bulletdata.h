// bulletdata.h
#pragma once
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>
#include <QDebug>
#include <vector>

struct Bullet {
    QString caliber;
    QString manufacturer;
    QString model;
    double weight_gr;
    double weight_g;
    double diameter_in;
    double diameter_mm;
    QVariantMap drag_coefficients;
    double drag_coefficient;  // Drag coefficient for this bullet
};

class BulletData {
public:
    static std::vector<Bullet> loadBulletData(const QString &filePath);
    static QJsonDocument readJsonFile(const QString &filePath);
};
