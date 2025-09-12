#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "dof3.h"
#include "dof6.h"
#include "modifiedpointmass.h"
#include "rungekutta.h"
#include "siacci.h"
#include "pejsa.h"
#include "modifiedeuler.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , useMetricUnits(true)
{
    ui->setupUi(this);

    // Populate algorithm combo box
    ui->algorithmComboBox->addItem("3DOF", QVariant::fromValue(BallisticsAlgorithm::DOF3));
    ui->algorithmComboBox->addItem("6DOF", QVariant::fromValue(BallisticsAlgorithm::DOF6));
    ui->algorithmComboBox->addItem("Modified Point Mass", QVariant::fromValue(BallisticsAlgorithm::ModifiedPointMass));
    ui->algorithmComboBox->addItem("Runge-Kutta", QVariant::fromValue(BallisticsAlgorithm::RungeKutta));
    ui->algorithmComboBox->addItem("Modified Euler", QVariant::fromValue(BallisticsAlgorithm::ModifiedEuler));
    ui->algorithmComboBox->addItem("Siacci", QVariant::fromValue(BallisticsAlgorithm::Siacci));
    ui->algorithmComboBox->addItem("Pejsa", QVariant::fromValue(BallisticsAlgorithm::Pejsa));

    // Connect signals and slots
    connect(ui->calculateButton, &QPushButton::clicked, this, &MainWindow::calculateTrajectory);
    connect(ui->exportButton, &QPushButton::clicked, this, &MainWindow::exportToCSV);
    connect(ui->unitComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onUnitChanged);
    connect(ui->saveProfileButton, &QPushButton::clicked, this, &MainWindow::saveProfile);
    connect(ui->loadProfileButton, &QPushButton::clicked, this, &MainWindow::loadProfile);
    connect(ui->algorithmComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onAlgorithmChanged);

    updateUnitLabels();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onAlgorithmChanged(int index) {
    currentAlgorithm = ui->algorithmComboBox->itemData(index).value<BallisticsAlgorithm>();
}

void MainWindow::onUnitChanged(int index) {
    useMetricUnits = (index == 0);
    updateUnitLabels();
}

void MainWindow::updateUnitLabels() {
    if (useMetricUnits) {
        ui->massLabel->setText("Mass (kg):");
        ui->diameterLabel->setText("Diameter (m):");
        ui->muzzleVelocityLabel->setText("Muzzle Velocity (m/s):");
        ui->windSpeedLabel->setText("Wind Speed (m/s):");
        ui->plot->xAxis->setLabel("Range (m)");
        ui->plot->yAxis->setLabel("Height (m)");
    } else {
        ui->massLabel->setText("Mass (lb):");
        ui->diameterLabel->setText("Diameter (in):");
        ui->muzzleVelocityLabel->setText("Muzzle Velocity (ft/s):");
        ui->windSpeedLabel->setText("Wind Speed (ft/s):");
        ui->plot->xAxis->setLabel("Range (ft)");
        ui->plot->yAxis->setLabel("Height (ft)");
    }
}

double MainWindow::convertToMetric(double value, const QString &unitType) {
    if (useMetricUnits) return value;
    if (unitType == "mass") return value * 0.453592; // lb to kg
    if (unitType == "length") return value * 0.0254; // in to m
    if (unitType == "velocity") return value * 0.3048; // ft/s to m/s
    return value;
}

double MainWindow::convertFromMetric(double value, const QString &unitType) {
    if (useMetricUnits) return value;
    if (unitType == "mass") return value / 0.453592; // kg to lb
    if (unitType == "length") return value / 0.0254; // m to in
    if (unitType == "velocity") return value / 0.3048; // m/s to ft/s
    return value;
}

void MainWindow::calculateTrajectory() {
    double mass = convertToMetric(ui->massEdit->text().toDouble(), "mass");
    double diameter = convertToMetric(ui->diameterEdit->text().toDouble(), "length");
    double muzzleVelocity = convertToMetric(ui->muzzleVelocityEdit->text().toDouble(), "velocity");
    double launchAngle = ui->launchAngleEdit->text().toDouble();
    double windSpeed = convertToMetric(ui->windSpeedEdit->text().toDouble(), "velocity");
    double windDirection = ui->windDirectionEdit->text().toDouble();
    double latitude = ui->latitudeEdit->text().toDouble();
    double dragCoeff = ui->dragCoeffEdit->text().toDouble();

    delete ballisticsModel;
    switch (currentAlgorithm) {
    case BallisticsAlgorithm::DOF3:
        ballisticsModel = new DOF3();
        break;
    case BallisticsAlgorithm::DOF6:
        ballisticsModel = new DOF6();
        break;
    case BallisticsAlgorithm::ModifiedPointMass:
        ballisticsModel = new ModifiedPointMass();
        break;
    case BallisticsAlgorithm::RungeKutta:
        ballisticsModel = new RungeKutta();
        break;
    case BallisticsAlgorithm::ModifiedEuler:
        ballisticsModel = new ModifiedEuler();
        break;
    case BallisticsAlgorithm::Siacci:
        ballisticsModel = new Siacci();
        break;
    case BallisticsAlgorithm::Pejsa:
        ballisticsModel = new Pejsa();
        break;
    }

    ballisticsModel->setParameters(mass, diameter, dragCoeff, muzzleVelocity, launchAngle, windSpeed, windDirection, latitude);

    double dt = 0.01;
    for (int i = 0; i < 1000; ++i) {
        ballisticsModel->step(dt);
    }

    // Plot the trajectory
    plotTrajectory(ballisticsModel->getTrajectory());
}

void MainWindow::plotTrajectory(const std::vector<std::array<double, 3>>& trajectory) {
    QVector<double> x, y;
    for (const auto& point : trajectory) {
        x << point[0]; // x-coordinate
        y << point[1]; // y-coordinate
    }

    ui->plot->clearGraphs();
    ui->plot->addGraph();
    ui->plot->graph(0)->setData(x, y);

    // Set axis labels based on unit system
    if (useMetricUnits) {
        ui->plot->xAxis->setLabel("Range (m)");
        ui->plot->yAxis->setLabel("Height (m)");
    } else {
        ui->plot->xAxis->setLabel("Range (ft)");
        ui->plot->yAxis->setLabel("Height (ft)");
    }

    ui->plot->rescaleAxes();
    ui->plot->replot();
}

void MainWindow::exportToCSV() {
    QString fileName = QFileDialog::getSaveFileName(
        this,
        "Save Trajectory Data",
        "",
        "CSV Files (*.csv)"
        );
    if (fileName.isEmpty()) {
        return;
    }

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "Error", "Could not open file for writing.");
        return;
    }

    QTextStream out(&file);

    // Write header with metadata
    out << "# Ballistic Trajectory Data\n";
    out << "# Algorithm: ";
    switch (currentAlgorithm) {
    case BallisticsAlgorithm::DOF3: out << "3DOF\n"; break;
    case BallisticsAlgorithm::DOF6: out << "6DOF\n"; break;
    case BallisticsAlgorithm::ModifiedPointMass: out << "Modified Point Mass\n"; break;
    case BallisticsAlgorithm::RungeKutta: out << "Runge-Kutta\n"; break;
    case BallisticsAlgorithm::ModifiedEuler: out << "Modified Euler\n"; break;
    case BallisticsAlgorithm::Siacci: out << "Siacci\n"; break;
    case BallisticsAlgorithm::Pejsa: out << "Pejsa\n"; break;
    }
    out << "# Unit System: " << (useMetricUnits ? "Metric" : "Imperial") << "\n";
    out << "# Parameters:\n";
    out << "# Mass: " << (useMetricUnits ? ui->massEdit->text() + " kg" : QString::number(ui->massEdit->text().toDouble() * 2.20462) + " lb") << "\n";
    out << "# Diameter: " << (useMetricUnits ? ui->diameterEdit->text() + " m" : QString::number(ui->diameterEdit->text().toDouble() * 39.3701) + " in") << "\n";
    out << "# Muzzle Velocity: " << (useMetricUnits ? ui->muzzleVelocityEdit->text() + " m/s" : QString::number(ui->muzzleVelocityEdit->text().toDouble() * 3.28084) + " ft/s") << "\n";
    out << "# Launch Angle: " << ui->launchAngleEdit->text() << " rad\n";
    out << "# Wind Speed: " << (useMetricUnits ? ui->windSpeedEdit->text() + " m/s" : QString::number(ui->windSpeedEdit->text().toDouble() * 3.28084) + " ft/s") << "\n";
    out << "# Wind Direction: " << ui->windDirectionEdit->text() << " rad\n";
    out << "# Latitude: " << ui->latitudeEdit->text() << " rad\n";
    out << "# Drag Coefficient: " << ui->dragCoeffEdit->text() << "\n";
    out << "#\n";

    // Write CSV header
    if (useMetricUnits) {
        out << "Time (s),Range (m),Height (m),Velocity (m/s),Velocity X (m/s),Velocity Y (m/s)\n";
    } else {
        out << "Time (s),Range (ft),Height (ft),Velocity (ft/s),Velocity X (ft/s),Velocity Y (ft/s)\n";
    }

    // Get trajectory data
    const auto& trajectory = ballisticsModel->getTrajectory();

    // Write trajectory data
    for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto& point = trajectory[i];
        double x = point[0];
        double y = point[1];
        double t = point[2];

        // Convert to imperial if needed
        double displayX = useMetricUnits ? x : x * 3.28084;
        double displayY = useMetricUnits ? y : y * 3.28084;

        // For velocity, we need to access the model's internal state or calculate it
        // This is a simplified approach; you may need to adjust based on your model
        double vx = 0.0, vy = 0.0, v = 0.0;
        if (i < trajectory.size() - 1) {
            double dx = trajectory[i+1][0] - x;
            double dy = trajectory[i+1][1] - y;
            double dt = trajectory[i+1][2] - t;
            vx = dx / dt;
            vy = dy / dt;
            v = sqrt(vx * vx + vy * vy);
        }

        double displayVx = useMetricUnits ? vx : vx * 3.28084;
        double displayVy = useMetricUnits ? vy : vy * 3.28084;
        double displayV = useMetricUnits ? v : v * 3.28084;

        out << t << "," << displayX << "," << displayY << ","
            << displayV << "," << displayVx << "," << displayVy << "\n";
    }

    file.close();
    QMessageBox::information(this, "Success", "Trajectory data exported successfully.");
}

void MainWindow::saveProfile() {
    QString fileName = QFileDialog::getSaveFileName(this, "Save Profile", "", "JSON Files (*.json)");
    if (fileName.isEmpty()) return;
    saveProfileToJson(fileName);
}

void MainWindow::loadProfile() {
    QString fileName = QFileDialog::getOpenFileName(this, "Load Profile", "", "JSON Files (*.json)");
    if (fileName.isEmpty()) return;
    loadProfileFromJson(fileName);
}

void MainWindow::saveProfileToJson(const QString &fileName) {
    QJsonObject profile;
    profile["mass"] = ui->massEdit->text();
    profile["diameter"] = ui->diameterEdit->text();
    profile["dragCoeff"] = ui->dragCoeffEdit->text();
    profile["muzzleVelocity"] = ui->muzzleVelocityEdit->text();
    profile["launchAngle"] = ui->launchAngleEdit->text();
    profile["windSpeed"] = ui->windSpeedEdit->text();
    profile["windDirection"] = ui->windDirectionEdit->text();
    profile["latitude"] = ui->latitudeEdit->text();
    profile["unitSystem"] = ui->unitComboBox->currentIndex();

    QJsonDocument doc(profile);
    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly)) {
        QMessageBox::warning(this, "Error", "Could not save profile.");
        return;
    }
    file.write(doc.toJson());
    file.close();
}

void MainWindow::loadProfileFromJson(const QString &fileName) {
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly)) {
        QMessageBox::warning(this, "Error", "Could not load profile.");
        return;
    }
    QByteArray data = file.readAll();
    file.close();

    QJsonDocument doc(QJsonDocument::fromJson(data));
    QJsonObject profile = doc.object();

    ui->massEdit->setText(profile["mass"].toString());
    ui->diameterEdit->setText(profile["diameter"].toString());
    ui->dragCoeffEdit->setText(profile["dragCoeff"].toString());
    ui->muzzleVelocityEdit->setText(profile["muzzleVelocity"].toString());
    ui->launchAngleEdit->setText(profile["launchAngle"].toString());
    ui->windSpeedEdit->setText(profile["windSpeed"].toString());
    ui->windDirectionEdit->setText(profile["windDirection"].toString());
    ui->latitudeEdit->setText(profile["latitude"].toString());
    ui->unitComboBox->setCurrentIndex(profile["unitSystem"].toInt());

    onUnitChanged(ui->unitComboBox->currentIndex());
}
