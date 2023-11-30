import wx
import configparser
import sys

app = wx.App(False)

def load_configuration():
    dialog = wx.FileDialog(None, "Wählen Sie eine Konfigurationsdatei", wildcard="Konfigurationsdateien (*.*)|*.*", style=wx.FD_OPEN | wx.FD_FILE_MUST_EXIST)

    if dialog.ShowModal() == wx.ID_CANCEL:
        return None  # Der Benutzer hat abgebrochen

    path = dialog.GetPath()
    config = configparser.ConfigParser()
    config.read(path)
    return config


global_config = load_configuration()
if global_config:
    FDIALOG = global_config.getboolean('Einstellungen', 'FDIALOG')
    ANIMATION = global_config.getint('Einstellungen', 'ANIMATION')
    RAD_ALGO = global_config.getint('Einstellungen', 'RAD_ALGO')
    MERGE = global_config.getint('Einstellungen', 'MERGE')
    TREE_ONLY = global_config.getint('Einstellungen', 'TREE_ONLY')
    AUTO_STEP = global_config.getint('Einstellungen', 'AUTO_STEP')
    OVERHANG_ANGLE = global_config.getint('Einstellungen', 'OVERHANG_ANGLE')
    MAX_BRANCH_LENGTH = global_config.getint('Einstellungen', 'MAX_BRANCH_LENGTH')
    GROUND = global_config.getfloat('Einstellungen', 'GROUND')
    GRID_STEP = global_config.getint('Einstellungen', 'GRID_STEP')
    POINT_OVERHANG_TOLERANCE = global_config.getfloat('Einstellungen', 'POINT_OVERHANG_TOLERANCE')
    K = global_config.getint('Einstellungen', 'K')
    RADIUS = global_config.getfloat('Einstellungen', 'RADIUS')
    RADIUS_FACTOR = global_config.getfloat('Einstellungen', 'RADIUS_FACTOR')
    MIN_AREA = global_config.getfloat('Einstellungen', 'MIN_AREA')
    MESH_SIZE_MAX = global_config.getfloat('Einstellungen', 'MESH_SIZE_MAX')  
    MESH_SIZE_MIN = global_config.getfloat('Einstellungen', 'MESH_SIZE_MIN')
    CROWN_RAD = global_config.getint('Einstellungen', 'CROWN_RAD')
    PT_SORT = global_config.getint('Einstellungen', 'PT_SORT')
    OVERHANG_ANGLE_EDGE = global_config.getint('Einstellungen', 'OVERHANG_ANGLE_EDGE')
    LIN_DEF = global_config.getfloat('Einstellungen', 'LIN_DEF')
    ANG_DEF = global_config.getfloat('Einstellungen', 'ANG_DEF')
    CSV_IMP = global_config.getint('Einstellungen', 'CSV_IMP')
    CALC_ADD_PTS = global_config.getboolean('Einstellungen', 'CALC_ADD_PTS')
    SHIFT_CONUS = global_config.getfloat('Einstellungen','SHIFT_CONUS')
 
else:
    print("Keine Konfigurationsdatei geladen. Beendigung des Programms")
    sys.exit(1)