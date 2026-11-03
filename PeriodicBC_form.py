from abaqusGui import *
import pbc_core


class PeriodicBCForm(AFXForm):
    def __init__(self, owner):
        AFXForm.__init__(self, owner)
        self.config = pbc_core.default_config()

    def getFirstDialog(self):
        from PeriodicBCDB import PeriodicBCDB
        return PeriodicBCDB(self)

    def okToCancel(self):
        return True
