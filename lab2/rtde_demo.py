from rtde_control import RTDEControlInterface as RTDEControl

rtde_c = RTDEControl("")

rtde_c.moveJ([0.0, -1.57, 0.0, -1.57, 0.0, 0.0], 1.0, 1.0)
