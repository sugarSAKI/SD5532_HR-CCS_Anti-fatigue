# SD5532_HR-CCS_Anti-fatigue
## Project Introduction
This is a school project in school of Design, the Hong Kong Polytechnic University. Our concept is to design an HRC system to give real-time support feedback to heritage painting and calligraphy restorer' arm based on the level of muscle workload and motion intention.
Our system main consists of two components:
1. An embedded wearable device with integrated sensors and Arduino;
2. A robotic arm controlled by python.
Notes: Communication is established between wearable device and robotic arm.

## File Information
### Folder
1. EMG_raw_data: examples of raw data we collected from EMG sensor in both situation of with and without support.
### File
1. Raw data collection
2. EMG_data_collection.ino: EMG raw data collection example.
3. EMG_data_collection.py: Save EMG raw data to csv file.
4. EMG_workload.ino: How to process EMG raw data to workload.
5. EMG_vertical_feedback.ino: Arduino part in vertical control sub-system.
6. EMG_vertical_feedback.py: Robotic arm part in vertical control sub-system.
7. Horizontal_feedback.ino: Arduino part in horizontal control sub-system.
8. Horizontal_feedback.py: Robotic arm part in horizontal control sub-system.
9. Anti_fatigue_system.ino: Arduino part in final version system, including workload detection, vertical motion detection and horizontal motion detection.
10. Anti_fatigue_system.py: Robotic arm part in final version system, including robotic arm control system and communication with external OLED (data visualization of workflow).
