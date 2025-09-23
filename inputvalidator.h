/**
 * @file inputvalidator.h
 * @brief Singleton class for managing default validation values from a JSON file.
 */

#ifndef INPUTVALIDATOR_H
#define INPUTVALIDATOR_H

#include <map>
#include <string>
#include <utility>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>
#include <QDebug>
#include <limits>

/**
 * @class InputValidator
 * @brief Singleton class that provides default minimum and maximum values for input validation.
 */
class InputValidator {
public:
    /**
     * @brief Get the singleton instance of InputValidator.
     * @return Reference to the singleton instance.
     */
    static InputValidator& getInstance() {
        static InputValidator instance;
        return instance;
    }

    /**
     * @brief Get the minimum and maximum values for a specific field.
     * @param fieldName The name of the field to get validation values for.
     * @return A pair containing the minimum and maximum values.
     */
    std::pair<double, double> getValidationRange(const std::string& fieldName) const {
        auto it = validationRanges.find(fieldName);
        if (it != validationRanges.end()) {
            return it->second;
        }
        // Return a default range if the field is not found
        return std::make_pair(0.0, std::numeric_limits<double>::max());
    }

    /**
     * @brief Get the minimum and maximum values for a specific field, with unit conversion.
     * @param fieldName The name of the field to get validation values for.
     * @param isMetric True if the current unit system is metric, false for imperial.
     * @return A pair containing the minimum and maximum values in the appropriate units.
     */
    std::pair<double, double> getValidationRange(const std::string& fieldName, bool isMetric) const {
        auto it = validationRanges.find(fieldName);
        if (it != validationRanges.end()) {
            double min = it->second.first;
            double max = it->second.second;

            // Convert from mm to appropriate units based on field type
            if (fieldName == "diameter" || fieldName == "scopeHeight") {
                if (!isMetric) {
                    // Convert from mm to inches
                    min = min / 25.4;
                    max = max / 25.4;
                } else {
                    // Keep in mm (no conversion needed)
                }
            }
            else if (fieldName == "mass") {
                if (!isMetric) {
                    // Convert from grams to grains (assuming JSON values are in grams)
                    min = min * 15.4324;
                    max = max * 15.4324;
                }
                // else keep in grams
            }
            else if (fieldName == "muzzleVelocity" || fieldName == "windSpeed") {
                if (!isMetric) {
                    // Convert from m/s to ft/s
                    min = min * 3.28084;
                    max = max * 3.28084;
                }
                // else keep in m/s
            }
            else if (fieldName == "tableMaxRange" || fieldName == "tableInterval" || fieldName == "zeroRange") {
                if (!isMetric) {
                    // Convert from meters to yards
                    min = min / 1000.0 * 1.09361;  // mm to m to yd
                    max = max / 1000.0 * 1.09361;  // mm to m to yd
                } else {
                    // Convert from mm to meters
                    min = min / 1000.0;
                    max = max / 1000.0;
                }
            }
            // For angles, no conversion needed as they're in degrees in both systems

            return std::make_pair(min, max);
        }
        // Return a default range if the field is not found
        return std::make_pair(0.0, std::numeric_limits<double>::max());
    }

    /**
     * @brief Get the description for a specific field.
     * @param fieldName The name of the field.
     * @return The description of the field.
     */
    std::string getFieldDescription(const std::string& fieldName) const {
        auto it = fieldDescriptions.find(fieldName);
        if (it != fieldDescriptions.end()) {
            return it->second;
        }
        return "";
    }

    /**
     * @brief Set custom validation range for a specific field.
     * @param fieldName The name of the field.
     * @param min The minimum valid value.
     * @param max The maximum valid value.
     */
    void setValidationRange(const std::string& fieldName, double min, double max) {
        validationRanges[fieldName] = std::make_pair(min, max);
    }

    /**
 * @brief Check if a value is valid for a specific field.
 * @param fieldName The name of the field.
 * @param value The value to validate.
 * @param isMetric True if the current unit system is metric, false for imperial.
 * @param allowEmpty If true, empty values are considered valid.
 * @return true if the value is valid, false otherwise.
 */
    bool isValid(const std::string& fieldName, double value, bool isMetric, bool allowEmpty = true) const {
        // If empty values are allowed and the value is NaN (which happens when converting empty string to double)
        if (allowEmpty && std::isnan(value)) {
            return true;
        }

        auto range = getValidationRange(fieldName, isMetric);
        return value >= range.first && value <= range.second;
    }

    /**
     * @brief Load validation rules from a JSON file.
     * @param filePath The path to the JSON file.
     * @return true if the file was loaded successfully, false otherwise.
     */
    bool loadFromJson(const QString& filePath) {
        QFile file(filePath);
        if (!file.open(QIODevice::ReadOnly)) {
            qWarning() << "Could not open validation rules file:" << filePath;
            return false;
        }

        QByteArray data = file.readAll();
        file.close();

        QJsonDocument doc = QJsonDocument::fromJson(data);
        if (doc.isNull() || !doc.isObject()) {
            qWarning() << "Invalid JSON format in validation rules file";
            return false;
        }

        QJsonObject root = doc.object();
        if (!root.contains("validation_rules") || !root["validation_rules"].isObject()) {
            qWarning() << "Invalid JSON format: 'validation_rules' object not found";
            return false;
        }

        QJsonObject rules = root["validation_rules"].toObject();
        for (auto it = rules.begin(); it != rules.end(); ++it) {
            QString key = it.key();
            QJsonObject rule = it.value().toObject();

            if (rule.contains("min") && rule.contains("max") && rule["min"].isDouble() && rule["max"].isDouble()) {
                // Store all values in mm (or appropriate base units) as per the JSON file
                double min = rule["min"].toDouble();
                double max = rule["max"].toDouble();
                validationRanges[key.toStdString()] = std::make_pair(min, max);
            }

            if (rule.contains("description") && rule["description"].isString()) {
                fieldDescriptions[key.toStdString()] = rule["description"].toString().toStdString();
            }
        }

        return true;
    }

    // Delete copy constructor and assignment operator to prevent copies
    InputValidator(const InputValidator&) = delete;
    InputValidator& operator=(const InputValidator&) = delete;

private:
    // Map to store validation ranges for each field (all values stored in mm or base units)
    std::map<std::string, std::pair<double, double>> validationRanges;

    // Map to store field descriptions
    std::map<std::string, std::string> fieldDescriptions;

    /**
     * @brief Private constructor to enforce singleton pattern.
     */
    InputValidator() {
        // Try to load from JSON file first
        bool loaded = loadFromJson(":/config/input_validation_rules.json");

        // If loading from resource failed, try to load from local file
        if (!loaded) {
            loaded = loadFromJson("input_validation_rules.json");
        }

        // If still not loaded, use default values
        if (!loaded) {
            qWarning() << "Failed to load validation rules from file, using default values";

            // Initialize default validation ranges (all values in mm or base units)
            validationRanges["mass"] = std::make_pair(0.1, 200.0);  // grams
            fieldDescriptions["mass"] = "Mass values (grams or grains)";

            validationRanges["diameter"] = std::make_pair(0.1, 200.0);  // mm
            fieldDescriptions["diameter"] = "Diameter values (mm or inches)";

            validationRanges["muzzleVelocity"] = std::make_pair(10.0, 2000.0);  // m/s
            fieldDescriptions["muzzleVelocity"] = "Muzzle velocity values (m/s or ft/s)";

            validationRanges["launchAngle"] = std::make_pair(-90.0, 90.0);  // degrees
            fieldDescriptions["launchAngle"] = "Launch angle values (degrees)";

            validationRanges["windSpeed"] = std::make_pair(0.0, 100.0);  // m/s
            fieldDescriptions["windSpeed"] = "Wind speed values (m/s or yd/s)";

            validationRanges["windDirection"] = std::make_pair(0.0, 360.0);  // degrees
            fieldDescriptions["windDirection"] = "Wind direction values (degrees)";

            validationRanges["latitude"] = std::make_pair(-90.0, 90.0);  // degrees
            fieldDescriptions["latitude"] = "Latitude values (degrees)";

            validationRanges["scopeHeight"] = std::make_pair(1.0, 500.0);  // mm
            fieldDescriptions["scopeHeight"] = "Scope height values (mm or inches)";

            validationRanges["dragCoefficient"] = std::make_pair(0.01, 5.0);
            fieldDescriptions["dragCoefficient"] = "Drag coefficient values";

            validationRanges["tableMaxRange"] = std::make_pair(1000.0, 5000000.0);  // mm
            fieldDescriptions["tableMaxRange"] = "Table max range values (m or yd)";

            validationRanges["tableInterval"] = std::make_pair(100.0, 1000000.0);  // mm
            fieldDescriptions["tableInterval"] = "Table interval values (m or yd)";

            validationRanges["zeroRange"] = std::make_pair(1000.0, 5000000.0);  // mm
            fieldDescriptions["zeroRange"] = "Zero range values (m or yd)";
        }
    }
};

#endif // INPUTVALIDATOR_H
