from threading import Thread
import GetData as gd
import ICP as icp
import VisualPoint as vp

point_cloud = []
transfomred = []

# construct the object 
get_data = gd.GetData()
icp = icp.ICP() 
visual_point = vp.VisualPoint()

def save_data():
    global point_cloud, get_data
    point_cloud = get_data.point
    print(point_cloud)

def run_icp():
    global point_cloud, transfomred, icp
    transfomred = icp.icp_algorithm(point_cloud)

def render_addpoint():
    global transfomred, visual_point
    visual_point.add_view(transfomred)


#visual_point.render()
while True:
    th1 = Thread(target = get_data.capture, args = ())
    th2 = Thread(target = save_data, args = ())
    th3 = Thread(target = run_icp, args = (icp))
    th4 = Thread(target = render_addpoint, args = (visual_point))
    th5 = Thread(target = visual_point.render, args = ())
    th1.start()
    th2.start()
    th3.start()
    th4.start()
    th5.start()
    th1.join()
    th2.join()
    th3.join()
    th4.join()
    th5.join()