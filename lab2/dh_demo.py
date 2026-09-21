from roboticstoolbox.models.DH import UR5

ur5 = UR5()

print(ur5)
ur5.plot(ur5.random_q())
