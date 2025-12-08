import math
import matplotlib.pyplot as plt
import fields2cover as f2c

def main():
    # ---------------------------------------------------------
    # 1. 基础设置 (Setup)
    # ---------------------------------------------------------
    # 定义机器人：宽度 0.5m，最小转弯半径 1.0m
    robot = f2c.Robot(0.5, 0.5) 
    robot.setCruiseVel(0.5) # 设定巡航速度 (可选)
    
    # 定义区域 (40m x 40m 的正方形)
    ring = f2c.LinearRing()
    ring.addPoint(f2c.Point(0.0, 0.0))
    ring.addPoint(f2c.Point(40.0, 0.0))
    ring.addPoint(f2c.Point(40.0, 40.0))
    ring.addPoint(f2c.Point(0.0, 40.0))
    ring.addPoint(f2c.Point(0.0, 0.0)) # 闭合
    
    cell = f2c.Cell()
    cell.addRing(ring)
    cells = f2c.Cells(cell)

    print("Step 1: 区域定义完成")

    # ---------------------------------------------------------
    # 2. 生成地头 (Headland Generator - HG)
    # ---------------------------------------------------------
    # 在边界留出 3倍 机器人宽度的距离用于转弯
    # 注意：Fields2Cover Python API 使用直接类名，不是命名空间
    hg = f2c.HG_Const_gen()
    no_hl = hg.generateHeadlands(cells, 3.0 * robot.getWidth())
    
    print("Step 2: 地头(Headland)生成完成")

    # ---------------------------------------------------------
    # 3. 生成平行覆盖线 (Swath Generator - SG)
    # ---------------------------------------------------------
    # 使用暴力搜索算法 (BruteForce) 生成覆盖线
    # math.pi 是参考角度 (0度或180度)，决定是横着扫还是竖着扫
    sg = f2c.SG_BruteForce()
    swaths = sg.generateSwaths(math.pi, robot.getCovWidth(), no_hl.getGeometry(0))
    
    print(f"Step 3: 生成了 {swaths.size()} 条覆盖线 (Swaths)")

    # ---------------------------------------------------------
    # 4. 路径排序 (Route Planner - RP)
    # ---------------------------------------------------------
    # 决定走的顺序：这里选择经典的"弓字形"(Boustrophedon)
    rp = f2c.RP_Boustrophedon()
    route = rp.genSortedSwaths(swaths)

    # ---------------------------------------------------------
    # 5. 生成平滑路径 (Path Planner - PP)
    # ---------------------------------------------------------
    # 将直线连接起来，生成带转弯的平滑路径 (Dubins 曲线)
    pp = f2c.PP_PathPlanning()
    dubins = f2c.PP_DubinsCurves() # 使用 Dubins 曲线转弯
    path = pp.planPath(robot, route, dubins)
    
    print(f"Step 5: 最终路径生成完成，包含 {path.size()} 个状态点")

    # ---------------------------------------------------------
    # 6. 数据提取与可视化 (Visualization)
    # ---------------------------------------------------------
    # 提取坐标用于画图
    x = []
    y = []
    for i in range(path.size()):
        # state.point 只有位置，state 还有速度、朝向等信息
        p = path.getState(i).point 
        x.append(p.getX())
        y.append(p.getY())

    # 画图
    plt.figure(figsize=(10, 10))
    # 画出原始边界
    bx = [0, 40, 40, 0, 0]
    by = [0, 0, 40, 40, 0]
    plt.plot(bx, by, 'k--', label='Field Boundary')
    
    # 画出路径
    plt.plot(x, y, 'r-', label='Robot Path')
    plt.scatter([x[0]], [y[0]], c='g', s=100, label='Start')
    plt.scatter([x[-1]], [y[-1]], c='b', s=100, label='End')
    
    plt.title("Fields2Cover - Boustrophedon Path")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

if __name__ == "__main__":
    main()