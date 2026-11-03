from abaqusGui import *
from abaqusConstants import ALL
from PeriodicBC_form import PeriodicBCForm

toolset = getAFXApp().getAFXMainWindow().getPluginToolset()
toolset.registerGuiMenuButton(
    buttonText='PeriodicBC...', object=PeriodicBCForm(toolset),
    messageId=AFXMode.ID_ACTIVATE, icon=None,
    kernelInitString='import PeriodicBC_kernel', applicableModules=ALL,
    version='1.0.0', author='PeriodicBC',
    description='Matching-mesh 3D periodic / quasi-2D generalized plane strain; tensile and creep paths.',
    helpUrl='')
