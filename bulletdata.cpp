#include "bulletdata.h"
#include <QFileDialog>

QJsonDocument BulletData::readJsonFile(const QString &filePath) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "Could not open JSON file:" << filePath;
        return QJsonDocument();
    }
    QByteArray data = file.readAll();
    file.close();
    return QJsonDocument::fromJson(data);
}

std::vector<Bullet> BulletData::loadBulletData(const QString &filePath) {
    std::vector<Bullet> bullets;
    QJsonDocument doc = readJsonFile(filePath);
    if (doc.isNull() || !doc.isObject()) {
        qWarning() << "Invalid JSON data";
        return bullets;
    }

    QJsonObject root = doc.object();
    if (!root.contains("bullets") || !root["bullets"].isArray()) {
        qWarning() << "Invalid JSON format: 'bullets' array not found";
        return bullets;
    }

    QJsonArray bulletArray = root["bullets"].toArray();
    for (const QJsonValue &bulletValue : bulletArray) {
        if (!bulletValue.isObject()) continue;

        QJsonObject bulletObj = bulletValue.toObject();
        Bullet bullet;
        bullet.caliber = bulletObj.value("caliber").toString("");
        bullet.manufacturer = bulletObj.value("manufacturer").toString("");
        bullet.model = bulletObj.value("model").toString("");
        bullet.weight_gr = bulletObj.value("weight_gr").toDouble(0.0);
        bullet.weight_g = bulletObj.value("weight_g").toDouble(0.0);
        bullet.diameter_in = bulletObj.value("diameter_in").toDouble(0.0);
        bullet.diameter_mm = bulletObj.value("diameter_mm").toDouble(0.0);

        if (bulletObj.contains("drag_coefficients") && bulletObj["drag_coefficients"].isObject()) {
            bullet.drag_coefficients = bulletObj["drag_coefficients"].toObject().toVariantMap();
        }

        bullets.push_back(bullet);
    }

    return bullets;
}
