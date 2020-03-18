from threading import Thread
import GetData as gd
import VisualPoint as vp


POINT = []

def sync(ob1, ob2):
    while True:
        # print("ob1: ", ob1.POINT)
        # print("ob2: ", ob2.POINT)
        ob2.POINT = ob1.POINT

if __name__ == "__main__":
    get_data = gd.GetData()
    visual_point = vp.VisualPoint()
    while True:
        th1 = Thread(target = get_data.capture, args = ())
        th2 = Thread(target = visual_point.render, args = ())
        th3 = Thread(target = sync, args=(get_data, visual_point))
        th1.start()
        th2.start()
        th3.start()
        th1.join()
        th2.join()
        th3.join()
        