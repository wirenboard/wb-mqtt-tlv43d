defineVirtualDevice("tlv493d", {
    title: "Magnetometer Sensor",
    cells: {
        X: {
            type: "value",
            value: 0,
            min: -200000,
            max: 200000,
            units: "µT"
        },
        Y: {
            type: "value",
            value: 0,
            min: -200000,
            max: 200000,
            units: "µT"
        },
        Z: {
            type: "value",
            value: 0,
            min: -200000,
            max: 200000,
            units: "µT"
        }
}
});

// Правило для преобразования значений в mT
defineRule("convert_to_mT", {
    whenChanged: ["tlv493d/X", "tlv493d/Y", "tlv493d/Z"],
    then: function(newValue, devName, cellName) {
        var mT_value = newValue / 1000;  // Делим на 1000 для перевода в mT
        if (cellName === "X") {
            dev["tlv493d"]["X_mT"] = mT_value;
        }
        if (cellName === "Y") {
            dev["tlv493d"]["Y_mT"] = mT_value;
        }
        if (cellName === "Z") {
            dev["tlv493d"]["Z_mT"] = mT_value;
        }
    }
});

// Логирование данных при изменении
defineRule("log_magnetometer_data", {
    whenChanged: ["tlv493d/X", "tlv493d/Y", "tlv493d/Z", "tlv493d/Temperature"],
    then: function(newValue, devName, cellName) {
        log(cellName + " updated: " + newValue);
    }
});