import QtQuick 2.9
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import WindAdjusterPlugin 1.0

GridLayout {
    id: windControlLayout
    columns: 3
    rowSpacing: 10
    anchors.fill: parent
    anchors.margins: 10

    // X Velocity
    Text { text: "X Velocity" }
    Slider {
        id: xSlider
        from: -50
        to: 50
        stepSize: 1
        value: windControl.windVelocity.x
        onValueChanged: windControl.setWindVelocity(value, ySlider.value, zSlider.value)
    }
    Text { text: xSlider.value.toFixed(2) + " m/s" }

    // Y Velocity
    Text { text: "Y Velocity" }
    Slider {
        id: ySlider
        from: -50
        to: 50
        stepSize: 1
        value: windControl.windVelocity.y
        onValueChanged: windControl.setWindVelocity(xSlider.value, value, zSlider.value)
    }
    Text { text: ySlider.value.toFixed(2) + " m/s" }

    // Z Velocity
    Text { text: "Z Velocity" }
    Slider {
        id: zSlider
        from: -50
        to: 50
        stepSize: 1
        value: windControl.windVelocity.z
        onValueChanged: windControl.setWindVelocity(xSlider.value, ySlider.value, value)
    }
    Text { text: zSlider.value.toFixed(2) + " m/s" }

    // Create an instance of the WindAdjusterPlugin with a unique id
    WindAdjusterPlugin {
        id: windControl  // Ensure this id is unique
    }
    
    // Helper function for safe access
    function safeComponent(val) {
        return val !== undefined ? val : 0;
    }
}