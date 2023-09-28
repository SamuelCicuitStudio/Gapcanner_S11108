// Function to calculate the gap width
uint16_t calculateGapWidth(uint16_t SensorData[]) {
    // Initialize minVal and maxVal with the first element of the array
    uint16_t minVal = SensorData[0];
    uint16_t maxVal = SensorData[0];

    // Calculate the minimum and maximum values in SensorData[]
    for (int i = 0; i < NUM_PIXELS; i++) {
        if (SensorData[i] < minVal) {
            minVal = SensorData[i];
        }
        if (SensorData[i] > maxVal) {
            maxVal = SensorData[i];
        }
    }



    int highThreshold = maxVal - (maxVal - minVal) * thresholdPercentage / 100;
    int lowThreshold = minVal + (maxVal - minVal) * thresholdPercentage / 100;

    // Initialize variables for gap width calculation
    int gapWidth = 0;
    bool firstRisingEdgeDetected = false; // Flag to track the first rising edge
    int firstRisingEdgeIndex = 0; // Index of the first rising edge
     int secondRisingEdgeIndex = 0; // Index of the first rising edge
    int startingPoint;
   // Minimum and maximum gap widths to consider as valid gaps
    int minValidGapWidth = 71; // Minimum gap width in pixels
    int maxValidGapWidth = 428; // Maximum gap width in pixels


    // Iterate through SensorData
    for (int i = 0; i < NUM_PIXELS; i++) {
        if (!firstRisingEdgeDetected && SensorData[i] < lowThreshold && SensorData[i+1] > highThreshold) {
            // First rising edge detected
            firstRisingEdgeDetected = true;
            firstRisingEdgeIndex = i;
        };
         if (firstRisingEdgeDetected && SensorData[i] < lowThreshold && SensorData[i+1] > highThreshold) {
            // Second rising edge detected
            secondRisingEdgeIndex = i;
             startingPoint = secondRisingEdgeIndex - 100;
             Serial.print("starting at :");
             Serial.println(startingPoint);
            
        }
    }

    // Initialize variables to count low pixels to the left and right
    int lowPixelsLeft = 0;
    int lowPixelsRight = 0;

    // Initialize flag to track if we have encountered the first high pixel
    bool firstHighPixelEncountered = false;
    // Iterate through SensorData starting from the middle
    for (int i = startingPoint; i >= 0; i--) {
        if (SensorData[i] > highThreshold) {
            // High pixel value detected
            firstHighPixelEncountered = true;
            break; // Exit the loop
        } else {
            // Low pixel value detected
            lowPixelsLeft++;
        }
    }

    // Reset flag for the right side
    firstHighPixelEncountered = false;

    // Iterate through SensorData starting from the middle
    for (int i = startingPoint + 1; i < NUM_PIXELS; i++) {
        if (SensorData[i] > highThreshold) {
            // High pixel value detected
            firstHighPixelEncountered = true;
            break; // Exit the loop
        } else {
            // Low pixel value detected
            lowPixelsRight++;
        }
    }

    // Calculate the gap width in micrometers (µm)
    int pixelSize = 14; // 14 µm per pixel
    int gapWidthMicrometers = (lowPixelsLeft + lowPixelsRight) * pixelSize;

    return gapWidthMicrometers; // Return the gap width in µm
}

