/**
 * @file mainwindow.cpp
 * @brief Implementation of the MainWindow class for the Ballistics Calculator application.
 */

#include <QCoreApplication>
#include <QFileDialog>
#include <QStandardPaths>

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "inputvalidator.h"
#include "dof3.h"
#include "dof6.h"
#include "modifiedpointmass.h"
#include "rungekutta.h"
#include "siacci.h"
#include "pejsa.h"
#include "modifiedeuler.h"

// Constructor
/**
 * @brief Constructs a MainWindow object.
 *
 * Initializes the UI, sets up connections between signals and slots,
 * and prepares the application for use.
 *
 * @param parent The parent widget.
 */
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , useMetricUnits(true)
    , scopeHeight(0.038)  // Default scope height in meters (38mm)
    , hasUnsavedChanges(false)
    , currentDragModel("G7")  // Default to G7
    , dropUnit(DropUnit::Inches)
{
    ui->setupUi(this);

    ballisticsModel = nullptr;

    // Populate algorithm combo box
    ui->algorithmComboBox->addItem("3DOF", QVariant::fromValue(BallisticsAlgorithm::DOF3));
    ui->algorithmComboBox->addItem("6DOF", QVariant::fromValue(BallisticsAlgorithm::DOF6));
    ui->algorithmComboBox->addItem("Modified Point Mass", QVariant::fromValue(BallisticsAlgorithm::ModifiedPointMass));
    ui->algorithmComboBox->addItem("Runge-Kutta", QVariant::fromValue(BallisticsAlgorithm::RungeKutta));
    ui->algorithmComboBox->addItem("Modified Euler", QVariant::fromValue(BallisticsAlgorithm::ModifiedEuler));
    ui->algorithmComboBox->addItem("Siacci", QVariant::fromValue(BallisticsAlgorithm::Siacci));
    ui->algorithmComboBox->addItem("Pejsa", QVariant::fromValue(BallisticsAlgorithm::Pejsa));

    // Set up drop unit combo box
    ui->dropUnitComboBox->addItem("Inches", QVariant::fromValue(DropUnit::Inches));
    ui->dropUnitComboBox->addItem("MOA", QVariant::fromValue(DropUnit::MOA));
    ui->dropUnitComboBox->addItem("MIL", QVariant::fromValue(DropUnit::MIL));

    // Connect signals and slots
    connect(ui->calculateButton, &QPushButton::clicked, this, &MainWindow::calculateTrajectory);
    connect(ui->exportButton, &QPushButton::clicked, this, &MainWindow::exportToCSV);
    connect(ui->unitComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onUnitChanged);
    connect(ui->saveProfileButton, &QPushButton::clicked, this, &MainWindow::saveProfile);
    connect(ui->loadProfileButton, &QPushButton::clicked, this, &MainWindow::loadProfile);
    connect(ui->algorithmComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onAlgorithmChanged);
    connect(ui->bulletComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onBulletSelected);
    connect(ui->loadBulletDataButton, &QPushButton::clicked, this, &MainWindow::loadBulletDataFile);
    connect(ui->calculateZeroButton, &QPushButton::clicked, this, &MainWindow::calculateZeroAngle);
    connect(ui->generateTableButton, &QPushButton::clicked, this, &MainWindow::generateTrajectoryTable);
    connect(ui->dropUnitComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onDropUnitChanged);
    connect(ui->actionExit, &QAction::triggered, this, &MainWindow::close);

    connect(ui->massEdit, &QLineEdit::editingFinished, this, [this]() {
        double temp;
        validateInput(ui->massEdit, temp, "mass", true);
        hasUnsavedChanges = true;
    });

    connect(ui->diameterEdit, &QLineEdit::editingFinished, this, [this]() {
        double temp;
        validateInput(ui->diameterEdit, temp, "diameter", true);
        hasUnsavedChanges = true;
    });

    connect(ui->muzzleVelocityEdit, &QLineEdit::editingFinished, this, [this]() {
        double temp;
        if (!validateInput(ui->muzzleVelocityEdit, temp, "muzzleVelocity", false)) {
            ui->muzzleVelocityEdit->setFocus();
            hasUnsavedChanges = true;
        }
    });

    connect(ui->launchAngleEdit, &QLineEdit::editingFinished, this, [this]() {
        QString text = ui->launchAngleEdit->text().trimmed();
        if (!text.isEmpty()) {
            bool ok;
            double angleDeg = text.toDouble(&ok);
            if (!ok || !InputValidator::getInstance().isValid("launchAngle", angleDeg, true, false)) {
                auto range = InputValidator::getInstance().getValidationRange("launchAngle", true);
                QMessageBox::warning(this, "Invalid Input",
                                     QString("Launch angle must be between %1 and %2 degrees.").arg(range.first).arg(range.second));
                ui->launchAngleEdit->setFocus();
                hasUnsavedChanges = true;
            }
        }
    });

    connect(ui->windSpeedEdit, &QLineEdit::editingFinished, this, [this]() {
        double temp;
        validateInput(ui->windSpeedEdit, temp, "windSpeed", true);
        hasUnsavedChanges = true;
    });

    connect(ui->windDirectionEdit, &QLineEdit::editingFinished, this, [this]() {
        QString text = ui->windDirectionEdit->text().trimmed();
        if (!text.isEmpty()) {
            bool ok;
            double windDirDeg = text.toDouble(&ok);
            if (!ok || !InputValidator::getInstance().isValid("windDirection", windDirDeg, true, false)) {
                auto range = InputValidator::getInstance().getValidationRange("windDirection", true);
                QMessageBox::warning(this, "Invalid Input",
                                     QString("Wind direction must be between %1 and %2 degrees.").arg(range.first).arg(range.second));
                ui->windDirectionEdit->setFocus();
            }
            hasUnsavedChanges = true;
        }
    });

    connect(ui->latitudeEdit, &QLineEdit::editingFinished, this, [this]() {
        QString text = ui->latitudeEdit->text().trimmed();
        if (!text.isEmpty()) {
            bool ok;
            double latitude = text.toDouble(&ok);
            if (!ok || !InputValidator::getInstance().isValid("latitude", latitude, true, false)) {
                auto range = InputValidator::getInstance().getValidationRange("latitude", true);
                QMessageBox::warning(this, "Invalid Input",
                                     QString("Latitude must be between %1 and %2 degrees.").arg(range.first).arg(range.second));
                ui->latitudeEdit->setFocus();
            }
            hasUnsavedChanges = true;
        }
    });

    connect(ui->scopeHeightEdit, &QLineEdit::editingFinished, this, [this]() {
        double temp;
        validateInput(ui->scopeHeightEdit, temp, "scopeHeight", true);
        hasUnsavedChanges = true;
    });

    connect(ui->dragCoeffEdit, &QLineEdit::editingFinished, this, [this]() {
        double temp;
        validateInput(ui->dragCoeffEdit, temp, "dragCoefficient", true);
        hasUnsavedChanges = true;
    });

    // For table parameters, we might want to keep them required
    connect(ui->tableMaxRangeEdit, &QLineEdit::editingFinished, this, [this]() {
        double temp;
        if (!validateInput(ui->tableMaxRangeEdit, temp, "tableMaxRange", false)) {
            ui->tableMaxRangeEdit->setFocus();
            hasUnsavedChanges = true;
        }
    });

    connect(ui->tableIntervalEdit, &QLineEdit::editingFinished, this, [this]() {
        double temp;
        if (!validateInput(ui->tableIntervalEdit, temp, "tableInterval", false)) {
            ui->tableIntervalEdit->setFocus();
            return;
        }

        // Check if interval is smaller than max range
        bool ok;
        double maxRange = ui->tableMaxRangeEdit->text().toDouble(&ok);
        double interval = ui->tableIntervalEdit->text().toDouble(&ok);

        if (ok && maxRange > 0 && interval >= maxRange) {
            QMessageBox::warning(this, "Invalid Input", "Interval must be smaller than maximum range.");
            ui->tableIntervalEdit->setFocus();
            hasUnsavedChanges = true;
        }
    });

    connect(ui->zeroRangeEdit, &QLineEdit::editingFinished, this, [this]() {
        double temp;
        if (!validateInput(ui->zeroRangeEdit, temp, "zeroRange", false)) {
            ui->zeroRangeEdit->setFocus();
            hasUnsavedChanges = true;
        }
    });


    // Connect input field changes to validation
    connect(ui->massEdit, &QLineEdit::textChanged, this, &MainWindow::onInputFieldChanged);
    connect(ui->diameterEdit, &QLineEdit::textChanged, this, &MainWindow::onInputFieldChanged);
    connect(ui->muzzleVelocityEdit, &QLineEdit::textChanged, this, &MainWindow::onInputFieldChanged);
    connect(ui->scopeHeightEdit, &QLineEdit::textChanged, this, &MainWindow::onInputFieldChanged);

    // Also connect other fields that might affect validation
    connect(ui->launchAngleEdit, &QLineEdit::textChanged, this, &MainWindow::onInputFieldChanged);
    connect(ui->windSpeedEdit, &QLineEdit::textChanged, this, &MainWindow::onInputFieldChanged);
    connect(ui->windDirectionEdit, &QLineEdit::textChanged, this, &MainWindow::onInputFieldChanged);
    connect(ui->latitudeEdit, &QLineEdit::textChanged, this, &MainWindow::onInputFieldChanged);
    connect(ui->dragCoeffEdit, &QLineEdit::textChanged, this, &MainWindow::onInputFieldChanged);

    // Initially disable the calculate button
    ui->calculateButton->setEnabled(false);
    ui->calculateZeroButton->setEnabled(false);
    ui->generateTableButton->setEnabled(false);

    // Set up drag model combo box
    ui->dragModelComboBox->addItem("G1");
    ui->dragModelComboBox->addItem("G7");
    ui->dragModelComboBox->setCurrentIndex(1);  // Default to G7

    // Connect drag model combo box
    connect(ui->dragModelComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onDragModelChanged);

    updateUnitLabels();
    setupInputValidators();

    // Set up tooltips
    setupTooltips();

    // Connect unit change to update tooltips
    connect(ui->unitComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::setupTooltips);
}

// Destructor
/**
 * @brief Destroys the MainWindow object.
 *
 * Cleans up the UI and any allocated resources.
 */
MainWindow::~MainWindow()
{
    delete ui;
}

/**
 * @brief Handles changes to the drag model selection.
 *
 * Updates the current drag model when the user selects a different model from the combo box.
 *
 * @param index The index of the selected drag model in the combo box.
 */
void MainWindow::onDragModelChanged(int index) {
    if (index == 0) {
        currentDragModel = "G1";
    } else {
        currentDragModel = "G7";
    }

    // Update the drag coefficient if needed
    validateInputsAndUpdateCalculateButton();
}

/**
 * @brief Validates all required inputs and updates the calculate button state.
 *
 * Checks all required input fields and enables/disables the calculate button
 * based on whether all required fields have valid values.
 */
void MainWindow::validateInputsAndUpdateCalculateButton() {
    bool allValid = true;

    // Check required fields
    double temp;

    // Mass is required
    if (!validateInput(ui->massEdit, temp, "mass", false)) {
        allValid = false;
    }

    // Diameter is required
    if (!validateInput(ui->diameterEdit, temp, "diameter", false)) {
        allValid = false;
    }

    // Muzzle velocity is required
    if (!validateInput(ui->muzzleVelocityEdit, temp, "muzzleVelocity", false)) {
        allValid = false;
    }

    // Scope height is required
    if (!validateInput(ui->scopeHeightEdit, temp, "scopeHeight", false)) {
        allValid = false;
    }

    // Enable/disable the calculate button based on validation
    ui->calculateButton->setEnabled(allValid);
    ui->calculateZeroButton->setEnabled(allValid);
    ui->generateTableButton->setEnabled(allValid);
}

/**
 * @brief Handles changes to input fields.
 *
 * Called when any input field is changed to trigger validation.
 */
void MainWindow::onInputFieldChanged() {
    validateInputsAndUpdateCalculateButton();
}

/**
 * @brief Sets up tooltips for all input fields.
 *
 * Creates tooltips with descriptions and valid ranges for all input fields,
 * taking into account the current unit system.
 */
void MainWindow::setupTooltips() {
    // Helper function to create tooltip text
    auto createTooltip = [this](const std::string& fieldName) -> QString {
        QString description = QString::fromStdString(InputValidator::getInstance().getFieldDescription(fieldName));
        auto range = InputValidator::getInstance().getValidationRange(fieldName, useMetricUnits);

        QString unit;
        if (fieldName == "mass") {
            unit = useMetricUnits ? "grams" : "grains";
        } else if (fieldName == "diameter") {
            unit = useMetricUnits ? "millimeters" : "inches";
        } else if (fieldName == "muzzleVelocity") {
            unit = useMetricUnits ? "meters/second" : "feet/second";
        } else if (fieldName == "windSpeed") {
            unit = useMetricUnits ? "meters/second" : "yards/second";
        } else if (fieldName == "scopeHeight") {
            unit = useMetricUnits ? "millimeters" : "inches";
        } else if (fieldName == "tableMaxRange" || fieldName == "tableInterval" || fieldName == "zeroRange") {
            unit = useMetricUnits ? "meters" : "yards";
        } else if (fieldName == "launchAngle" || fieldName == "windDirection" || fieldName == "latitude") {
            unit = "degrees";
        }

        QString tooltip = QString("%1\n\n").arg(description);

        if (fieldName != "launchAngle" && fieldName != "windDirection" && fieldName != "latitude") {
            tooltip += QString("Valid range: %1 to %2 %3")
            .arg(range.first)
                .arg(range.second)
                .arg(unit);
        } else {
            tooltip += QString("Valid range: %1 to %2 degrees")
            .arg(range.first)
                .arg(range.second);
        }

        // Set tooltip for drag model combo box
        ui->dragModelComboBox->setToolTip("Select the drag model to use for calculations.\n"
                                          "G1: Standard drag model for flat-based bullets.\n"
                                          "G7: Modern drag model for boat-tail bullets.");

        return tooltip;
    };

    // Set tooltips for all input fields
    ui->massEdit->setToolTip(createTooltip("mass"));
    ui->diameterEdit->setToolTip(createTooltip("diameter"));
    ui->muzzleVelocityEdit->setToolTip(createTooltip("muzzleVelocity"));
    ui->launchAngleEdit->setToolTip(createTooltip("launchAngle"));
    ui->windSpeedEdit->setToolTip(createTooltip("windSpeed"));
    ui->windDirectionEdit->setToolTip(createTooltip("windDirection"));
    ui->latitudeEdit->setToolTip(createTooltip("latitude"));
    ui->scopeHeightEdit->setToolTip(createTooltip("scopeHeight"));
    ui->dragCoeffEdit->setToolTip(createTooltip("dragCoefficient"));
    ui->tableMaxRangeEdit->setToolTip(createTooltip("tableMaxRange"));
    ui->tableIntervalEdit->setToolTip(createTooltip("tableInterval"));
    ui->zeroRangeEdit->setToolTip(createTooltip("zeroRange"));

    // Set tooltips for combo boxes
    ui->unitComboBox->setToolTip("Select the unit system: Metric or Imperial");
    ui->algorithmComboBox->setToolTip("Select the ballistics algorithm to use for calculations");
    ui->bulletComboBox->setToolTip("Select a bullet profile from the database");
    ui->dropUnitComboBox->setToolTip("Select the unit for displaying drop values: Inches, MOA, or MIL");

    // Set tooltips for buttons
    ui->calculateButton->setToolTip("Calculate the bullet trajectory with current parameters");
    ui->exportButton->setToolTip("Export the trajectory data to a CSV file");
    ui->saveProfileButton->setToolTip("Save the current parameters to a profile file");
    ui->loadProfileButton->setToolTip("Load parameters from a profile file");
    ui->loadBulletDataButton->setToolTip("Load bullet data from a JSON file");
    ui->calculateZeroButton->setToolTip("Calculate the zero angle for the specified range");
    ui->generateTableButton->setToolTip("Generate a trajectory table with the specified parameters");
}

/**
 * @brief Handles the close event when the user tries to close the window.
 *
 * Prompts the user to confirm exit and accepts or ignores the close event based on the user's choice.
 *
 * @param event The close event.
 */
void MainWindow::closeEvent(QCloseEvent *event) {
    // Check for unsaved changes if needed
    if (hasUnsavedChanges) {
        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(this, "Unsaved Changes",
                                      "You have unsaved changes. Do you want to save before exiting?",
                                      QMessageBox::Yes|QMessageBox::No|QMessageBox::Cancel);

        if (reply == QMessageBox::Cancel) {
            event->ignore();
            return;
        } else if (reply == QMessageBox::Yes) {
            // Call your save function here
            // saveProfile();
        }
    }

    // If no unsaved changes or user confirmed, close the application
    event->accept();
}

/**
 * @brief Sets up input validators with default values.
 */
void MainWindow::setupInputValidators() {
    // The singleton already has default values, but you can customize them here if needed
    // Example: InputValidator::getInstance().setValidationRange("mass", 0.5, 150.0);
}

/**
 * @brief Validates a single input field using the InputValidator singleton.
 *
 * @param field The QLineEdit field to validate.
 * @param value Reference to store the validated value.
 * @param fieldName The name of the field for validation.
 * @param allowEmpty If true, empty fields are considered valid.
 * @return true if the input is valid, false otherwise.
 */
bool MainWindow::validateInput(QLineEdit* field, double& value, const std::string& fieldName, bool allowEmpty) {
    QString text = field->text().trimmed();

    // If the field is empty and empty is allowed, return true
    if (allowEmpty && text.isEmpty()) {
        return true;
    }

    // If the field is empty but not allowed, return false
    if (text.isEmpty()) {
        return false;
    }

    bool ok;
    double val = text.toDouble(&ok);

    if (!ok) {
        return false;
    }

    // Check if the value is valid using the current unit system
    if (!InputValidator::getInstance().isValid(fieldName, val, useMetricUnits, allowEmpty)) {
        return false;
    }

    value = val;
    return true;
}

/**
 * @brief Validates all input fields using the InputValidator singleton.
 *
 * @return true if all inputs are valid, false otherwise.
 */
bool MainWindow::validateAllInputs() {
    double temp;

    // Validate mass (required)
    if (!validateInput(ui->massEdit, temp, "mass", false)) {
        QMessageBox::warning(this, "Invalid Input", "Please enter a valid mass.");
        ui->massEdit->setFocus();
        return false;
    }

    // Validate diameter (required)
    if (!validateInput(ui->diameterEdit, temp, "diameter", false)) {
        QMessageBox::warning(this, "Invalid Input", "Please enter a valid diameter.");
        ui->diameterEdit->setFocus();
        return false;
    }

    // Validate muzzle velocity (required)
    if (!validateInput(ui->muzzleVelocityEdit, temp, "muzzleVelocity", false)) {
        QMessageBox::warning(this, "Invalid Input", "Please enter a valid muzzle velocity.");
        ui->muzzleVelocityEdit->setFocus();
        return false;
    }

    // Validate scope height (required)
    if (!validateInput(ui->scopeHeightEdit, temp, "scopeHeight", false)) {
        QMessageBox::warning(this, "Invalid Input", "Please enter a valid scope height.");
        ui->scopeHeightEdit->setFocus();
        return false;
    }

    // Validate launch angle (not required)
    bool ok;
    QString angleText = ui->launchAngleEdit->text().trimmed();
    if (!angleText.isEmpty()) {
        double angleDeg = angleText.toDouble(&ok);
        if (!ok || !InputValidator::getInstance().isValid("launchAngle", angleDeg, true, false)) {
            auto range = InputValidator::getInstance().getValidationRange("launchAngle", true);
            QMessageBox::warning(this, "Invalid Input",
                                 QString("Launch angle must be between %1 and %2 degrees.").arg(range.first).arg(range.second));
            ui->launchAngleEdit->setFocus();
            return false;
        }
    }

    // Validate wind speed (not required)
    if (!validateInput(ui->windSpeedEdit, temp, "windSpeed", true)) {
        // Don't return false for optional fields
    }

    // Validate wind direction (not required)
    QString windDirText = ui->windDirectionEdit->text().trimmed();
    if (!windDirText.isEmpty()) {
        double windDirDeg = windDirText.toDouble(&ok);
        if (!ok || !InputValidator::getInstance().isValid("windDirection", windDirDeg, true, false)) {
            auto range = InputValidator::getInstance().getValidationRange("windDirection", true);
            QMessageBox::warning(this, "Invalid Input",
                                 QString("Wind direction must be between %1 and %2 degrees.").arg(range.first).arg(range.second));
            ui->windDirectionEdit->setFocus();
            return false;
        }
    }

    // Validate latitude (not required)
    QString latitudeText = ui->latitudeEdit->text().trimmed();
    if (!latitudeText.isEmpty()) {
        double latitude = latitudeText.toDouble(&ok);
        if (!ok || !InputValidator::getInstance().isValid("latitude", latitude, true, false)) {
            auto range = InputValidator::getInstance().getValidationRange("latitude", true);
            QMessageBox::warning(this, "Invalid Input",
                                 QString("Latitude must be between %1 and %2 degrees.").arg(range.first).arg(range.second));
            ui->latitudeEdit->setFocus();
            return false;
        }
    }

    // Validate drag coefficient (not required)
    if (!validateInput(ui->dragCoeffEdit, temp, "dragCoefficient", true)) {
        // Don't return false for optional fields
    }

    return true;
}

/**
 * @brief Handles changes to the drop unit selection.
 *
 * Updates the drop unit when the user selects a different unit from the combo box.
 *
 * @param index The index of the selected drop unit in the combo box.
 */
void MainWindow::onDropUnitChanged(int index) {
    if (index < 0) return;

    // No validation needed for drop unit change, just update
    dropUnit = ui->dropUnitComboBox->itemData(index).value<DropUnit>();

    // Regenerate the table if it exists
    if (ui->trajectoryTable->rowCount() > 0) {
        bool ok;
        double maxRange = ui->tableMaxRangeEdit->text().toDouble(&ok);
        if (!ok || maxRange <= 0) return;

        double interval = ui->tableIntervalEdit->text().toDouble(&ok);
        if (!ok || interval <= 0) return;

        // Convert to meters if in imperial mode
        if (!useMetricUnits) {
            maxRange *= 0.9144; // yards to meters
            interval *= 0.9144; // yards to meters
        }

        populateTrajectoryTable(maxRange, interval);
    }
}

void MainWindow::calculateZeroAngle() {
    double temp;

    // Validate zero range (required)
    if (!validateInput(ui->zeroRangeEdit, temp, "zeroRange", false)) {
        QMessageBox::warning(this, "Invalid Input", "Please enter a valid zero range.");
        ui->zeroRangeEdit->setFocus();
        return;
    }
    double range = temp;

    // Convert range to meters if in imperial mode
    if (!useMetricUnits) {
        range *= 0.9144; // yards to meters
    }

    calculateAndDisplayZeroAngle(range);
}

void MainWindow::generateTrajectoryTable() {
    double temp;

    // Validate max range (required)
    if (!validateInput(ui->tableMaxRangeEdit, temp, "tableMaxRange", false)) {
        QMessageBox::warning(this, "Invalid Input", "Please enter a valid maximum range.");
        ui->tableMaxRangeEdit->setFocus();
        return;
    }
    double maxRange = temp;

    // Validate interval (required)
    if (!validateInput(ui->tableIntervalEdit, temp, "tableInterval", false)) {
        QMessageBox::warning(this, "Invalid Input", "Please enter a valid interval.");
        ui->tableIntervalEdit->setFocus();
        return;
    }
    double interval = temp;

    // Check if interval is smaller than max range
    if (interval >= maxRange) {
        QMessageBox::warning(this, "Error", "Interval must be smaller than maximum range.");
        ui->tableIntervalEdit->setFocus();
        return;
    }

    // Convert to meters if in imperial mode
    if (!useMetricUnits) {
        maxRange *= 0.9144; // yards to meters
        interval *= 0.9144; // yards to meters
    }

    populateTrajectoryTable(maxRange, interval);

    // Switch to the Trajectory Table tab
    ui->tabWidget->setCurrentIndex(3);
}

/**
 * @brief Populates the trajectory table with calculated values.
 *
 * @param maxRange The maximum range to show in the table.
 * @param interval The interval between rows in the table.
 */
void MainWindow::populateTrajectoryTable(double maxRange, double interval) {
    int bulletIndex = ui->bulletComboBox->currentIndex();
    if (bulletIndex < 0 || static_cast<size_t>(bulletIndex) >= bulletDatabase.size()) {
        QMessageBox::warning(this, "Error", "Please select a bullet profile.");
        return;
    }

    Bullet selectedBullet = bulletDatabase[static_cast<size_t>(bulletIndex)];
    double mass = convertToMetric(ui->massEdit->text().toDouble(), "mass");
    double diameter = convertToMetric(ui->diameterEdit->text().toDouble(), "length");
    double muzzleVelocity = convertToMetric(ui->muzzleVelocityEdit->text().toDouble(), "velocity");
    double launchAngle = ui->launchAngleEdit->text().toDouble();
    double windSpeed = convertToMetric(ui->windSpeedEdit->text().toDouble(), "windSpeed");
    double windDirection = ui->windDirectionEdit->text().toDouble();
    double latitude = ui->latitudeEdit->text().toDouble();

    // Get drag coefficient based on selected model
    double dragCoeff = getDragCoefficient(selectedBullet, muzzleVelocity, "G7");

    // Clear and set up the table
    ui->trajectoryTable->clear();
    ui->trajectoryTable->setRowCount(0);
    ui->trajectoryTable->setColumnCount(9);

    // Set table headers based on units
    QStringList headers;
    if (useMetricUnits) {
        headers << "Range (m)" << "Height (m)" << "Height Above Scope (m)" << "Time (s)"
                << "Velocity (m/s)" << "Energy (J)" << "Drop (";

        // Add drop unit to header
        if (dropUnit == DropUnit::Inches) headers.last() += "in)";
        else if (dropUnit == DropUnit::MOA) headers.last() += "MOA)";
        else headers.last() += "MIL)";

        headers << "Windage (m)" << "Vertical Velocity (m/s)" << "Horizontal Velocity (m/s)";
    } else {
        headers << "Range (yd)" << "Height (yd)" << "Height Above Scope (yd)" << "Time (s)"
                << "Velocity (ft/s)" << "Energy (ft-lb)" << "Drop (";

        // Add drop unit to header
        if (dropUnit == DropUnit::Inches) headers.last() += "in)";
        else if (dropUnit == DropUnit::MOA) headers.last() += "MOA)";
        else headers.last() += "MIL)";

        headers << "Windage (in)" << "Vertical Velocity (ft/s)" << "Horizontal Velocity (ft/s)";
    }
    ui->trajectoryTable->setHorizontalHeaderLabels(headers);

    // Create a temporary ballistics model for table generation
    BallisticsModel* tableModel = new DOF6();
    tableModel->setParameters(mass, diameter, dragCoeff, muzzleVelocity, launchAngle, windSpeed, windDirection, latitude, scopeHeight);

    // Generate trajectory data
    double dt = 0.01;
    std::vector<std::array<double, 3>> fullTrajectory;
    for (int i = 0; i < 10000; ++i) {  // Increased iterations for longer ranges
        tableModel->step(dt);
        const auto& trajectory = tableModel->getTrajectory();
        if (!trajectory.empty()) {
            fullTrajectory = trajectory;
        }
        // Stop if the bullet has hit the ground (y < 0)
        if (!trajectory.empty() && trajectory.back()[1] < 0) {
            break;
        }
    }

    // Find the maximum height for drop calculation
    double maxHeight = 0;
    for (size_t i = 0; i < fullTrajectory.size(); ++i) {
        if (fullTrajectory[i][1] > maxHeight) {
            maxHeight = fullTrajectory[i][1];
        }
    }

    // Add rows to the table at specified intervals
    int row = 0;
    for (double range = 0; range <= maxRange; range += interval) {
        ui->trajectoryTable->insertRow(row);

        // Find the trajectory point closest to the current range
        double closestX = 0, closestY = 0, closestT = 0;
        size_t closestIndex = 0;
        double minDist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < fullTrajectory.size(); ++i) {
            double dist = std::abs(fullTrajectory[i][0] - range);
            if (dist < minDist) {
                minDist = dist;
                closestX = fullTrajectory[i][0];
                closestY = fullTrajectory[i][1];
                closestT = fullTrajectory[i][2];
                closestIndex = i;
            }
        }

        // Calculate velocity components
        double vx = 0.0, vy = 0.0, v = 0.0;
        if (closestIndex > 0 && closestIndex < fullTrajectory.size()) {
            double dt = fullTrajectory[closestIndex][2] - fullTrajectory[closestIndex-1][2];
            if (dt > 0) {
                vx = (fullTrajectory[closestIndex][0] - fullTrajectory[closestIndex-1][0]) / dt;
                vy = (fullTrajectory[closestIndex][1] - fullTrajectory[closestIndex-1][1]) / dt;
                v = std::sqrt(vx*vx + vy*vy);
            }
        }

        // Calculate energy (E = 0.5 * m * v^2)
        double energy = 0.5 * mass * v * v;  // in Joules

        // Calculate drop (difference between max height and current height)
        double drop = maxHeight - closestY;

        // Calculate windage (lateral displacement due to wind)
        // This is a simplified calculation - actual windage would require more complex modeling
        double windage = 0.0;
        if (windSpeed > 0) {
            // Simple approximation: windage increases with range and wind speed
            windage = 0.001 * closestX * windSpeed * std::sin(windDirection);
        }

        // Convert values to display units
        double displayRange = useMetricUnits ? closestX : closestX / 0.9144;
        double displayHeight = useMetricUnits ? closestY : closestY / 0.9144;
        double displayV = useMetricUnits ? v : v / 0.3048;
        double displayVx = useMetricUnits ? vx : vx / 0.3048;
        double displayVy = useMetricUnits ? vy : vy / 0.3048;
        double displayDrop = useMetricUnits ? drop : drop / 0.0254;  // meters to inches for drop
        double displayWindage = useMetricUnits ? windage : windage / 0.0254;  // meters to inches for windage
        double displayEnergy = useMetricUnits ? energy : energy * 0.737562;  // Joules to ft-lb

        // Calculate height above scope (current height + scope height)
        double heightAboveScope = closestY + scopeHeight;
        double displayHeightAboveScope = useMetricUnits ? heightAboveScope : heightAboveScope / 0.9144;

        // Convert drop to the selected unit
        if (useMetricUnits) {
            if (dropUnit == DropUnit::Inches) {
                displayDrop = drop / 0.0254;  // meters to inches
            } else if (dropUnit == DropUnit::MOA) {
                // MOA = (drop in inches) / (range in yards) * 100
                // First convert drop to inches and range to yards
                displayDrop = (drop / 0.0254) / (closestX / 0.9144) * 100;
            } else { // MIL
                // MIL = (drop in meters) / (range in meters) * 1000
                displayDrop = (drop / closestX) * 1000;
            }
        } else {
            if (dropUnit == DropUnit::Inches) {
                displayDrop = drop / 0.0254;  // meters to inches
            } else if (dropUnit == DropUnit::MOA) {
                // MOA = (drop in inches) / (range in yards) * 100
                displayDrop = (drop / 0.0254) / (closestX / 0.9144) * 100;
            } else { // MIL
                // Convert range to yards and drop to inches for display, but calculate MIL in meters
                displayDrop = (drop / (closestX * 0.9144 / 0.9144)) * 1000;  // Simplified for display
                // Actually, MIL is (drop in meters)/(range in meters)*1000, so we need to convert properly:
                displayDrop = (drop / closestX) * 1000;
            }
        }

        // Add items to the table
        ui->trajectoryTable->setItem(row, 0, new QTableWidgetItem(QString::number(displayRange, 'f', 1)));
        ui->trajectoryTable->setItem(row, 1, new QTableWidgetItem(QString::number(displayHeight, 'f', 2)));
        ui->trajectoryTable->setItem(row, 2, new QTableWidgetItem(QString::number(displayHeightAboveScope, 'f', 2)));
        ui->trajectoryTable->setItem(row, 3, new QTableWidgetItem(QString::number(closestT, 'f', 2)));
        ui->trajectoryTable->setItem(row, 4, new QTableWidgetItem(QString::number(displayV, 'f', 2)));
        ui->trajectoryTable->setItem(row, 5, new QTableWidgetItem(QString::number(displayEnergy, 'f', 1)));
        ui->trajectoryTable->setItem(row, 6, new QTableWidgetItem(QString::number(displayDrop, 'f', 2)));
        ui->trajectoryTable->setItem(row, 7, new QTableWidgetItem(QString::number(displayWindage, 'f', 2)));
        ui->trajectoryTable->setItem(row, 8, new QTableWidgetItem(QString::number(displayVy, 'f', 1)));
        ui->trajectoryTable->setItem(row, 9, new QTableWidgetItem(QString::number(displayVx, 'f', 1)));

        row++;
    }
}

/**
 * @brief Converts drop in meters to the selected drop unit.
 *
 * @param drop The drop value in meters.
 * @param range The range in meters.
 * @return The drop value in the selected unit.
 */
double MainWindow::convertDropToDisplayUnit(double drop, double range) const {
    if (dropUnit == DropUnit::Inches) {
        return drop / 0.0254;  // meters to inches
    } else if (dropUnit == DropUnit::MOA) {
        // MOA = (drop in inches) / (range in yards) * 100
        return (drop / 0.0254) / (range / 0.9144) * 100;
    } else { // MIL
        // MIL = (drop in meters) / (range in meters) * 1000
        return (drop / range) * 1000;
    }
}

/**
 * @brief Converts a value in MOA to meters.
 *
 * @param moa The value in MOA.
 * @param range The range in meters.
 * @return The value in meters.
 */
double MainWindow::moaToMeters(double moa, double range) const {
    // MOA to inches: moa_value * range_in_yards / 100
    // Then convert inches to meters: inches * 0.0254
    double rangeInYards = range / 0.9144;
    double inches = moa * rangeInYards / 100;
    return inches * 0.0254;
}

/**
 * @brief Converts a value in MIL to meters.
 *
 * @param mil The value in MIL.
 * @param range The range in meters.
 * @return The value in meters.
 */
double MainWindow::milToMeters(double mil, double range) const {
    // MIL to meters: mil_value * range_in_meters / 1000
    return mil * range / 1000;
}

/**
 * @brief Calculates and displays the zero angle for a given range.
 *
 * @param range The range at which to calculate the zero angle.
 */
void MainWindow::calculateAndDisplayZeroAngle(double range) {
    int bulletIndex = ui->bulletComboBox->currentIndex();
    if (bulletIndex < 0 || static_cast<size_t>(bulletIndex) >= bulletDatabase.size()) {
        QMessageBox::warning(this, "Error", "Please select a bullet profile.");
        return;
    }

    Bullet selectedBullet = bulletDatabase[static_cast<size_t>(bulletIndex)];
    double mass = convertToMetric(ui->massEdit->text().toDouble(), "mass");
    double diameter = convertToMetric(ui->diameterEdit->text().toDouble(), "length");
    double muzzleVelocity = convertToMetric(ui->muzzleVelocityEdit->text().toDouble(), "velocity");
    double windSpeed = convertToMetric(ui->windSpeedEdit->text().toDouble(), "windSpeed");
    double windDirection = ui->windDirectionEdit->text().toDouble();
    double latitude = ui->latitudeEdit->text().toDouble();

    // Get drag coefficient based on selected model
    double dragCoeff = getDragCoefficient(selectedBullet, muzzleVelocity, "G7");

    // Create a temporary ballistics model for zero angle calculation
    BallisticsModel* zeroModel = new DOF6(); // Using 6DOF for zero angle calculation
    zeroModel->setParameters(mass, diameter, dragCoeff, muzzleVelocity, 0, windSpeed, windDirection, latitude, scopeHeight);

    // Binary search to find the zero angle
    double lowAngle = -0.1; // -0.1 radians
    double highAngle = 0.5;  // 0.5 radians (about 28.6 degrees)
    double bestAngle = 0.0;
    double minError = std::numeric_limits<double>::max();
    const double tolerance = 0.0001; // 0.0001 radians tolerance
    const int maxIterations = 100;

    for (int i = 0; i < maxIterations; ++i) {
        double midAngle1 = lowAngle + (highAngle - lowAngle) / 3.0;
        double midAngle2 = highAngle - (highAngle - lowAngle) / 3.0;

        // Test midAngle1
        zeroModel->setParameters(mass, diameter, dragCoeff, muzzleVelocity, midAngle1, windSpeed, windDirection, latitude, scopeHeight);
        double dt = 0.01;
        double currentX = 0.0;
        double currentY = 0.0;
        for (int step = 0; step < 5000; ++step) {
            zeroModel->step(dt);
            const auto& trajectory = zeroModel->getTrajectory();
            if (!trajectory.empty()) {
                currentX = trajectory.back()[0];
                currentY = trajectory.back()[1];
            }
            if (currentX >= range) {
                break;
            }
        }

        double error1 = std::abs(currentY);

        // Test midAngle2
        zeroModel->setParameters(mass, diameter, dragCoeff, muzzleVelocity, midAngle2, windSpeed, windDirection, latitude, scopeHeight);
        currentX = 0.0;
        currentY = 0.0;
        for (int step = 0; step < 5000; ++step) {
            zeroModel->step(dt);
            const auto& trajectory = zeroModel->getTrajectory();
            if (!trajectory.empty()) {
                currentX = trajectory.back()[0];
                currentY = trajectory.back()[1];
            }
            if (currentX >= range) {
                break;
            }
        }

        double error2 = std::abs(currentY);

        if (error1 < minError) {
            minError = error1;
            bestAngle = midAngle1;
        }
        if (error2 < minError) {
            minError = error2;
            bestAngle = midAngle2;
        }

        if (error1 < error2) {
            highAngle = midAngle2;
        } else {
            lowAngle = midAngle1;
        }

        if (std::abs(highAngle - lowAngle) < tolerance) {
            break;
        }
    }

    delete zeroModel;

    // Convert angle to degrees for display
    double zeroAngleDegrees = bestAngle * 180.0 / M_PI;

    // Display the zero angle
    ui->zeroAngleValue->setText(QString::number(zeroAngleDegrees, 'f', 2) + "°");

    // Optionally set this angle as the launch angle
    ui->launchAngleEdit->setText(QString::number(bestAngle, 'f', 4));
}

/**
 * @brief Loads bullet data from a JSON file.
 *
 * Opens a file dialog for the user to select a bullet data file,
 * then loads the bullet profiles from the JSON file.
 */
void MainWindow::loadBulletDataFile() {
    // Get the directory of the executable
    QString executablePath = QCoreApplication::applicationDirPath();

    // Set up the file dialog
    QString fileName = QFileDialog::getOpenFileName(
        this,
        tr("Open Bullet Data File"),
        executablePath,  // Start in the executable directory
        tr("JSON Files (*.json);;All Files (*)"));

    if (!fileName.isEmpty()) {
        QFile file(fileName);
        if (!file.open(QIODevice::ReadOnly)) {
            QMessageBox::warning(this, "Error", "Could not open file for reading.");
            return;
        }

        QByteArray data = file.readAll();
        file.close();

        QJsonDocument doc = QJsonDocument::fromJson(data);
        if (doc.isNull() || !doc.isObject()) {
            QMessageBox::warning(this, "Error", "Invalid JSON format.");
            return;
        }

        QJsonObject root = doc.object();
        if (!root.contains("bullets") || !root["bullets"].isArray()) {
            QMessageBox::warning(this, "Error", "Invalid bullet data format. Expected 'bullets' array.");
            return;
        }

        QJsonArray bullets = root["bullets"].toArray();
        bulletDatabase.clear();

        for (const QJsonValue& bulletValue : bullets) {
            if (!bulletValue.isObject()) continue;

            QJsonObject bulletObj = bulletValue.toObject();
            Bullet bullet;

            // Parse bullet data
            if (bulletObj.contains("model") && bulletObj["model"].isString()) {
                bullet.model = bulletObj["model"].toString();
            }

            if (bulletObj.contains("weight_g") && bulletObj["weight_g"].isDouble()) {
                bullet.weight_g = bulletObj["weight_g"].toDouble();
            }

            if (bulletObj.contains("diameter_mm") && bulletObj["diameter_mm"].isDouble()) {
                bullet.diameter_mm = bulletObj["diameter_mm"].toDouble();
            }

            if (bulletObj.contains("drag_coefficient") && bulletObj["drag_coefficient"].isDouble()) {
                bullet.drag_coefficient = bulletObj["drag_coefficient"].toDouble();
            }

            // Add more fields as needed

            bulletDatabase.push_back(bullet);
        }

        // Update the bullet combo box
        populateBulletComboBox();

        QMessageBox::information(this, "Success", "Bullet data loaded successfully.");
    }
}

/**
 * @brief Populates the bullet combo box with available bullet profiles.
 *
 * Clears the current items and adds new items for each bullet profile in the database.
 */
void MainWindow::populateBulletComboBox() {
    ui->bulletComboBox->clear();

    if (bulletDatabase.empty()) {
        ui->bulletComboBox->addItem("No bullet profiles loaded");
        return;
    }

    for (const auto& bullet : bulletDatabase) {
        ui->bulletComboBox->addItem(bullet.model);
    }
}

/**
 * @brief Handles changes to the selected bullet profile.
 *
 * Updates the mass and diameter fields when the user selects a different bullet profile.
 *
 * @param index The index of the selected bullet in the combo box.
 */
void MainWindow::onBulletSelected(int index) {
    if (index < 0 || static_cast<size_t>(index) >= bulletDatabase.size()) return;

    Bullet selectedBullet = bulletDatabase[static_cast<size_t>(index)];
    if (useMetricUnits) {
        ui->massEdit->setText(QString::number(selectedBullet.weight_g, 'f', 2));
        ui->diameterEdit->setText(QString::number(selectedBullet.diameter_mm, 'f', 2));
    } else {
        ui->massEdit->setText(QString::number(selectedBullet.weight_gr, 'f', 1));
        ui->diameterEdit->setText(QString::number(selectedBullet.diameter_in, 'f', 3));
    }
}

/**
 * @brief Gets the drag coefficient for a bullet at a specific velocity.
 *
 * @param bullet The bullet for which to get the drag coefficient.
 * @param velocity The velocity at which to get the drag coefficient.
 * @param model The drag model to use ("G1" or "G7").
 * @return The drag coefficient at the specified velocity.
 */
double MainWindow::getDragCoefficient(const Bullet &bullet, double velocity, const QString &model) {
    // If the bullet has a specific drag coefficient, use that
    if (bullet.drag_coefficient > 0) {
        return bullet.drag_coefficient;
    }

    // Otherwise, use standard drag curves based on the selected model
    double mach = velocity / 343.0;  // Speed of sound in m/s

    if (model == "G1") {
        // G1 standard drag curve
        if (mach < 0.95) {
            return 0.295;  // Subsonic G1
        } else if (mach < 1.2) {
            return 0.295 + (0.220 - 0.295) * (mach - 0.95) / 0.25;  // Transonic
        } else {
            return 0.220;  // Supersonic G1
        }
    } else {  // G7
        // G7 standard drag curve (more appropriate for modern bullets)
        if (mach < 0.95) {
            return 0.150;  // Subsonic G7
        } else if (mach < 1.2) {
            return 0.150 + (0.120 - 0.150) * (mach - 0.95) / 0.25;  // Transonic
        } else {
            return 0.120;  // Supersonic G7
        }
    }
}

/**
 * @brief Handles changes to the selected ballistics algorithm.
 *
 * Updates the current algorithm when the user selects a different one from the combo box.
 *
 * @param index The index of the selected algorithm in the combo box.
 */
void MainWindow::onAlgorithmChanged(int index) {
    currentAlgorithm = ui->algorithmComboBox->itemData(index).value<BallisticsAlgorithm>();
}

/**
 * @brief Handles changes to the unit system selection.
 *
 * Updates all unit labels when the user changes between metric and imperial units.
 *
 * @param index The index of the selected unit system in the combo box.
 */
void MainWindow::onUnitChanged(int index) {
    // Check if we're actually changing the unit system
    bool newMetric = (index == 0);
    if (newMetric == useMetricUnits) return;

    // Validate current inputs before changing units
    if (!validateAllInputs()) {
        // Revert the combo box selection
        ui->unitComboBox->setCurrentIndex(useMetricUnits ? 0 : 1);
        return;
    }

    useMetricUnits = newMetric;
    updateUnitLabels();
    updateZeroAngleLabels();
}

/**
 * @brief Updates the zero angle labels based on the selected unit system.
 *
 * Changes the text of the zero angle labels to reflect either metric or imperial units.
 */
void MainWindow::updateZeroAngleLabels() {
    if (useMetricUnits) {
        ui->zeroRangeLabel->setText("Zero Range (m):");
    } else {
        ui->zeroRangeLabel->setText("Zero Range (yd):");
    }
}

/**
 * @brief Updates the unit labels based on the selected unit system.
 *
 * Changes the text of all unit labels to reflect either metric or imperial units.
 * Also updates the plot axis labels and refreshes the plot.
 */
void MainWindow::updateUnitLabels() {
    if (useMetricUnits) {
        ui->massLabel->setText("Mass (g):");
        ui->diameterLabel->setText("Diameter (mm):");
        ui->muzzleVelocityLabel->setText("Muzzle Velocity (m/s):");
        ui->windSpeedLabel->setText("Wind Speed (m/s):");
        ui->zeroRangeLabel->setText("Zero Range (m):");
        ui->tableMaxRangeLabel->setText("Max Range (m):");
        ui->tableIntervalLabel->setText("Interval (m):");
        ui->scopeHeightLabel->setText("Scope Height (mm):");

        // Update plot axis labels
        if (ui->plot) {
            ui->plot->xAxis->setLabel("Range (m)");
            ui->plot->yAxis->setLabel("Height (m)");
            ui->plot->replot();
        }
    } else {
        ui->massLabel->setText("Mass (gr):");
        ui->diameterLabel->setText("Diameter (in):");
        ui->muzzleVelocityLabel->setText("Muzzle Velocity (ft/s):");
        ui->windSpeedLabel->setText("Wind Speed (yd/s):");
        ui->zeroRangeLabel->setText("Zero Range (yd):");
        ui->tableMaxRangeLabel->setText("Max Range (yd):");
        ui->tableIntervalLabel->setText("Interval (yd):");
        ui->scopeHeightLabel->setText("Scope Height (in):");

        // Update plot axis labels
        if (ui->plot) {
            ui->plot->xAxis->setLabel("Range (yd)");
            ui->plot->yAxis->setLabel("Height (yd)");
            ui->plot->replot();
        }
    }
}

/**
 * @brief Converts a value from the current unit system to metric units.
 *
 * @param value The value to convert.
 * @param unitType The type of unit (e.g., "mass", "length", "velocity").
 * @return The value converted to metric units.
 */
double MainWindow::convertToMetric(double value, const QString &unitType) {
    if (useMetricUnits) return value;
    if (unitType == "mass") return value / 15.4324; // grains to grams
    if (unitType == "length") return value * 0.0254; // inches to meters
    if (unitType == "range") return value * 0.9144; // yards to meters
    if (unitType == "velocity") return value * 0.3048; // ft/s to m/s
    if (unitType == "windSpeed") return value * 0.9144; // yd/s to m/s
    return value;
}

/**
 * @brief Converts a value from metric units to the current unit system.
 *
 * @param value The value to convert.
 * @param unitType The type of unit (e.g., "mass", "length", "velocity").
 * @return The value converted to the current unit system.
 */
double MainWindow::convertFromMetric(double value, const QString &unitType) {
    if (useMetricUnits) return value;
    if (unitType == "mass") return value * 15.4324; // grams to grains
    if (unitType == "length") return value / 0.0254; // meters to inches
    if (unitType == "range") return value / 0.9144; // meters to yards
    if (unitType == "velocity") return value / 0.3048; // m/s to ft/s
    if (unitType == "windSpeed") return value / 0.9144; // m/s to yd/s
    return value;
}

/**
 * @brief Gets the drag coefficient for a bullet at a specific velocity.
 *
 * @param bullet The bullet for which to get the drag coefficient.
 * @param velocity The velocity at which to get the drag coefficient.
 * @param model The drag model to use (e.g., "G1", "G7").
 * @return The drag coefficient at the specified velocity.
 */
double MainWindow::getDragCoefficientAtVelocity(const Bullet &bullet, double velocity, const QString &model)
{
    QVariantMap dragData = bullet.drag_coefficients.value(model).toMap();
    if (dragData.isEmpty()) return 0.5; // Default value

    // If we have a constant BC, use it
    if (dragData.contains("bc")) {
        double bc = dragData["bc"].toDouble();
        double diameter = bullet.diameter_mm / 1000.0; // Convert to meters
        double mass = bullet.weight_g / 1000.0; // Convert to kg
        return (mass / (diameter * diameter)) / bc;
    }

    // If we have speed-dependent coefficients, find the closest
    QVariantList supersonicData = dragData["supersonic"].toList();
    if (supersonicData.isEmpty()) return 0.5; // Default value

    // Convert velocity to ft/s for comparison with the table
    double velocityFtPerSec = velocity / 0.3048;

    // Find the closest velocity range
    double closestCd = supersonicData[0].toMap()["cd"].toDouble();
    double minDiff = std::abs(supersonicData[0].toMap()["velocity"].toDouble() - velocityFtPerSec);

    for (const QVariant &entry : supersonicData) {
        QVariantMap data = entry.toMap();
        double entryVelocity = data["velocity"].toDouble();
        double diff = std::abs(entryVelocity - velocityFtPerSec);
        if (diff < minDiff) {
            minDiff = diff;
            closestCd = data["cd"].toDouble();
        }
    }

    return closestCd;
}

void MainWindow::calculateTrajectory() {
    // Validate all inputs before proceeding
    if (!validateAllInputs()) {
        return;
    }

    int bulletIndex = ui->bulletComboBox->currentIndex();
    if (bulletIndex < 0 || static_cast<size_t>(bulletIndex) >= bulletDatabase.size()) {
        QMessageBox::warning(this, "Error", "Please select a bullet profile.");
        return;
    }

    // Get scope height from UI and convert to meters
    double scopeHeightInput = ui->scopeHeightEdit->text().toDouble();
    if (useMetricUnits) {
        scopeHeight = scopeHeightInput / 1000.0;  // Convert mm to m
    } else {
        scopeHeight = scopeHeightInput * 0.0254;  // Convert in to m
    }

    Bullet selectedBullet = bulletDatabase[static_cast<size_t>(bulletIndex)];

    // Get mass with fallback to bullet profile if empty
    double mass;
    if (ui->massEdit->text().trimmed().isEmpty()) {
        mass = convertToMetric(selectedBullet.weight_g, "mass");
    } else {
        mass = convertToMetric(ui->massEdit->text().toDouble(), "mass");
    }

    // Get diameter with fallback to bullet profile if empty
    double diameter;
    if (ui->diameterEdit->text().trimmed().isEmpty()) {
        diameter = convertToMetric(selectedBullet.diameter_mm / 1000.0, "length");
    } else {
        diameter = convertToMetric(ui->diameterEdit->text().toDouble(), "length");
    }

    // Get muzzle velocity (required)
    double muzzleVelocity = convertToMetric(ui->muzzleVelocityEdit->text().toDouble(), "velocity");

    // Get launch angle (default to 0 if empty)
    double launchAngle = 0.0;
    if (!ui->launchAngleEdit->text().trimmed().isEmpty()) {
        launchAngle = ui->launchAngleEdit->text().toDouble() * M_PI / 180.0;  // Convert degrees to radians
    }

    // Get wind speed (default to 0 if empty)
    double windSpeed = 0.0;
    if (!ui->windSpeedEdit->text().trimmed().isEmpty()) {
        windSpeed = convertToMetric(ui->windSpeedEdit->text().toDouble(), "windSpeed");
    }

    // Get wind direction (default to 0 if empty)
    double windDirection = 0.0;
    if (!ui->windDirectionEdit->text().trimmed().isEmpty()) {
        windDirection = ui->windDirectionEdit->text().toDouble() * M_PI / 180.0;  // Convert degrees to radians
    }

    // Get latitude (default to 0 if empty)
    double latitude = 0.0;
    if (!ui->latitudeEdit->text().trimmed().isEmpty()) {
        latitude = ui->latitudeEdit->text().toDouble() * M_PI / 180.0;  // Convert degrees to radians
    }

    // Get drag coefficient with fallback to default if empty
    double dragCoeff;
    if (ui->dragCoeffEdit->text().trimmed().isEmpty()) {
        // Use a default drag coefficient if not specified
        dragCoeff = 0.5;  // Default value
    } else {
        dragCoeff = ui->dragCoeffEdit->text().toDouble();
    }

    // Get drag coefficient based on selected model
    dragCoeff = getDragCoefficient(selectedBullet, muzzleVelocity, currentDragModel);

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

    // Pass scopeHeight to setParameters
    ballisticsModel->setParameters(mass, diameter, dragCoeff, muzzleVelocity, launchAngle,
                                   windSpeed, windDirection, latitude, scopeHeight);

    double dt = 0.01;
    for (int i = 0; i < 1000; ++i) {
        ballisticsModel->step(dt);
    }

    plotTrajectory(ballisticsModel->getTrajectory());

    // Switch to the Trajectory Visualization tab after calculation
    ui->tabWidget->setCurrentIndex(1);
}

/**
 * @brief Plots the trajectory on the graph.
 *
 * @param trajectory A vector of points representing the bullet's trajectory.
 */
void MainWindow::plotTrajectory(const std::vector<std::array<double, 3>>& trajectory) {
    QVector<double> x, y;
    for (const auto& point : trajectory) {
        double displayX = useMetricUnits ? point[0] : point[0] / 0.9144; // meters to yards
        double displayY = useMetricUnits ? point[1] : point[1] / 0.9144; // meters to yards
        x << displayX;
        y << displayY;
    }

    // Clear all graphs
    ui->plot->clearGraphs();

    // Add graph for the trajectory
    ui->plot->addGraph();
    ui->plot->graph(0)->setData(x, y);
    ui->plot->graph(0)->setPen(QPen(Qt::blue));
    ui->plot->graph(0)->setName("Trajectory");

    // Add graph for the scope line (horizontal line at y=0)
    QVector<double> scopeX = x;
    QVector<double> scopeY;
    for (qsizetype i = 0; i < x.size(); ++i) {
        scopeY << 0;  // Scope line is at y=0 in the plot (which represents the scope height above barrel)
    }
    ui->plot->addGraph();
    ui->plot->graph(1)->setData(scopeX, scopeY);
    ui->plot->graph(1)->setPen(QPen(Qt::red));
    ui->plot->graph(1)->setName("Scope Line");

    // Set axis labels based on unit system
    if (useMetricUnits) {
        ui->plot->xAxis->setLabel("Range (m)");
        ui->plot->yAxis->setLabel("Height (m)");
    } else {
        ui->plot->xAxis->setLabel("Range (yd)");
        ui->plot->yAxis->setLabel("Height (yd)");
    }

    // Set axis ranges with some padding
    double xMin = 0;
    double xMax = x.isEmpty() ? 100 : *std::max_element(x.constBegin(), x.constEnd()) * 1.05;
    double yMin = y.isEmpty() ? -0.1 : *std::min_element(y.constBegin(), y.constEnd()) * 1.05;
    double yMax = y.isEmpty() ? 0.1 : *std::max_element(y.constBegin(), y.constEnd()) * 1.05;

    // Adjust yMin and yMax to include scope height in the view
    if (!useMetricUnits) {
        // Convert scope height to yards for display
        double displayScopeHeight = scopeHeight / 0.9144;
        yMin = std::min(yMin, -displayScopeHeight * 1.1);
        yMax = std::max(yMax, displayScopeHeight * 1.1);
    } else {
        // Scope height is already in meters
        yMin = std::min(yMin, -scopeHeight * 1.1);
        yMax = std::max(yMax, scopeHeight * 1.1);
    }

    ui->plot->xAxis->setRange(xMin, xMax);
    ui->plot->yAxis->setRange(yMin, yMax);

    ui->plot->replot();
}

/**
 * @brief Exports the trajectory data to a CSV file.
 *
 * Opens a file dialog for the user to select a location to save the CSV file,
 * then writes the trajectory data along with metadata to the file.
 */
void MainWindow::exportToCSV() {
    QString fileName = QFileDialog::getSaveFileName(this, "Save Trajectory Data", "", "CSV Files (*.csv)");
    if (fileName.isEmpty()) return;

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
    out << "# Mass: " << (useMetricUnits ? ui->massEdit->text() + " g" : ui->massEdit->text() + " gr") << "\n";
    out << "# Diameter: " << (useMetricUnits ? ui->diameterEdit->text() + " mm" : ui->diameterEdit->text() + " in") << "\n";
    out << "# Muzzle Velocity: " << (useMetricUnits ? ui->muzzleVelocityEdit->text() + " m/s" : ui->muzzleVelocityEdit->text() + " ft/s") << "\n";
    out << "# Launch Angle: " << ui->launchAngleEdit->text() << " rad\n";
    out << "# Wind Speed: " << (useMetricUnits ? ui->windSpeedEdit->text() + " m/s" : ui->windSpeedEdit->text() + " yd/s") << "\n";
    out << "# Wind Direction: " << ui->windDirectionEdit->text() << " rad\n";
    out << "# Latitude: " << ui->latitudeEdit->text() << " rad\n";
    out << "# Drag Coefficient: " << ui->dragCoeffEdit->text() << "\n";
    out << "#\n";

    // Write CSV header
    if (useMetricUnits) {
        out << "Time (s),Range (m),Height (m),Velocity (m/s),Velocity X (m/s),Velocity Y (m/s)\n";
    } else {
        out << "Time (s),Range (yd),Height (yd),Velocity (ft/s),Velocity X (ft/s),Velocity Y (ft/s)\n";
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
        double displayX = useMetricUnits ? x : x / 0.9144; // meters to yards
        double displayY = useMetricUnits ? y : y / 0.9144; // meters to yards

        // For velocity, calculate from trajectory points
        double vx = 0.0, vy = 0.0, v = 0.0;
        if (i < trajectory.size() - 1) {
            double dx = trajectory[i+1][0] - x;
            double dy = trajectory[i+1][1] - y;
            double dt = trajectory[i+1][2] - t;
            vx = dx / dt;
            vy = dy / dt;
            v = sqrt(vx * vx + vy * vy);
        }

        double displayVx = useMetricUnits ? vx : vx / 0.3048; // m/s to ft/s
        double displayVy = useMetricUnits ? vy : vy / 0.3048; // m/s to ft/s
        double displayV = useMetricUnits ? v : v / 0.3048; // m/s to ft/s

        out << t << "," << displayX << "," << displayY << ","
            << displayV << "," << displayVx << "," << displayVy << "\n";
    }

    file.close();
    QMessageBox::information(this, "Success", "Trajectory data exported successfully.");
}

/**
 * @brief Saves the current profile to a JSON file.
 *
 * Opens a file dialog for the user to select a location to save the profile,
 * then writes the current parameters to a JSON file.
 */
void MainWindow::saveProfile() {
    QString fileName = QFileDialog::getSaveFileName(this, "Save Profile", "", "JSON Files (*.json)");
    if (fileName.isEmpty()) return;
    saveProfileToJson(fileName);
    hasUnsavedChanges = false;
}

/**
 * @brief Loads a profile from a JSON file.
 *
 * Opens a file dialog for the user to select a profile file,
 * then loads the parameters from the JSON file.
 */
void MainWindow::loadProfile() {
    QString fileName = QFileDialog::getOpenFileName(this, "Load Profile", "", "JSON Files (*.json)");
    if (fileName.isEmpty()) return;
    loadProfileFromJson(fileName);
}

/**
 * @brief Saves the current profile to a JSON file.
 *
 * @param fileName The path to the file where the profile will be saved.
 */
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

/**
 * @brief Loads a profile from a JSON file.
 *
 * @param fileName The path to the file containing the profile to load.
 */
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
