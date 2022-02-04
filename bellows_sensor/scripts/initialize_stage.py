from stage_py2 import StageControl

axis = "THETA_STAGE"

if __name__ == "__main__":
    t_stage = StageControl(comport=b"/dev/ttyUSB0", baudrate=9600)

    t_stage.absolute_move(axis=axis, mm=0, speed=30)
     
