from roboticstoolbox.backends.PyPlot import PyPlot
from roboticstoolbox.models.URDF.URDFRobot import URDFRobot

ur5 = URDFRobot("ur5")
ur5.plot(ur5.random_q(), backend="swift")
# pyplot = PyPlot()
# pyplot.launch()
# pyplot.add(ur5)
# ur5.q = ur5.random_q()
# pyplot.step()
# pyplot.hold()
