# UKF-Based Bicycle State Estimator

## Code Files Included:

1. **main**: This is the main file, that will run the UKF estimator, record the outputs, and generate some figures.
2. **estInitialize**:  This function is called before any data arrives, and its purpose is to initialize the estimator. The function returns an object called ‘internalState’.
3. **estRun**: This function is run at every time instant, and has multiple arguments, including timing, the estimator internal state (in the same format as is returned by the estInitialize), information on the current steering angle and pedal speed, and the latest measurement.

## 📄 Full Report

This repo implements a UKF-based bicycle state estimator. For full theoretical background and implementation details, please see:

➡️ [Project Report (PDF)](./C231B_Project_Report.pdf)
