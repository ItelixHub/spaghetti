import wx
from file_exporter import write_step, write_stl

def exit_app(event):
    wx.Exit()

class MenuManager:
    def __init__(self, frame, display):
        self.frame = frame
        self.display = display
        self.menu_bar = wx.MenuBar()

    def create_menu(self):
        best_tree_menu = wx.Menu()

        write_step_item = best_tree_menu.Append(wx.ID_ANY, "Write STEP")
        self.frame.Bind(wx.EVT_MENU, lambda event: write_step(self.display), write_step_item)

        write_stl_item = best_tree_menu.Append(wx.ID_ANY, "Write STL")
        self.frame.Bind(wx.EVT_MENU, lambda event: write_stl(self.display), write_stl_item)

        exit_item = best_tree_menu.Append(wx.ID_EXIT, "Exit")
        self.frame.Bind(wx.EVT_MENU, exit_app, exit_item)

        self.menu_bar.Append(best_tree_menu, "BEST Tree")
        self.frame.SetMenuBar(self.menu_bar)

    def start(self):
        self.create_menu()
        self.frame.Show()
