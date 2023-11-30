# spaghetti
spaghetti-code
tree.py (Main Application)

This is the main file of the application.
It initializes the graphical user interface using wxPython and sets up the 3D viewer for the display of 3D models.
It contains the core logic for 3D object manipulation, including functions for creating, modifying, and analyzing 3D structures.
It handles user inputs and interactions with the application.
config_manager.py (Configuration Manager)

Manages configuration settings for the application.
Loads and parses configuration settings from a file, making these settings available to other parts of the application.
Ensures that the application behaves according to user-defined preferences or default settings.
csv_manager.py (CSV Manager)

Handles the import and processing of data from CSV files.
Provides functionality to read CSV files and convert data into a format usable by the application, particularly for 3D modeling.
file_exporter.py (File Exporter)

Manages the export of 3D models to various file formats such as STEP and STL.
Contains functions to save the current state of 3D models into these formats, allowing for external use and further processing.
menu_manager.py (Menu Manager)

Responsible for creating and managing the menu interface of the application.
Contains functions to add menus and menu items to the application's graphical user interface, linking these items to their respective functionalities.
