import wx
import numpy as np

def import_csv_data(app_instance):
    csv_dialog = wx.FileDialog(None, "Wählen Sie eine CSV-Datei", wildcard="CSV-Dateien (*.csv)|*.csv", style=wx.FD_OPEN | wx.FD_FILE_MUST_EXIST)
    
    if csv_dialog.ShowModal() == wx.ID_CANCEL:
        print("CSV-Import abgebrochen. Fortfahren ohne CSV-Import.")
        return np.array([]), None, 0
    else:
        csv_file_path = csv_dialog.GetPath()
        print(f"Ausgewählte CSV-Datei: {csv_file_path}")
        csv_data = np.genfromtxt(csv_file_path, delimiter=',', usecols=(0, 1, 2))
        return csv_data, csv_file_path, 1

def manage_csv_import(app, csv_imp_flag):
    if csv_imp_flag == 1:
        csv_data, csv_file_path, new_csv_imp_flag = import_csv_data(app)
        return csv_data, csv_file_path, new_csv_imp_flag
    else:
        print("Keine Konfiguration verfügbar.")
        return np.array([]), None, csv_imp_flag

