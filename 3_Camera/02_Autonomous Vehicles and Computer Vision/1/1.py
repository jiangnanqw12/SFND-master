import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 定义物体坐标
x = np.linspace(-1, 1, 50)
y = np.linspace(-1, 1, 50)
X, Y = np.meshgrid(x, y)
Z = np.zeros_like(X)

# 定义小孔位置
x_hole = 0
y_hole = 0
z_hole = 5

# 定义成像平面位置
x_image = np.linspace(-1, 1, 10)
y_image = np.linspace(-1, 1, 10)
X_image, Y_image = np.meshgrid(x_image, y_image)
Z_image = np.ones_like(X_image) * z_hole

# 计算光线经过小孔后在成像平面上的位置
X_prime = -z_hole * X / (Z - z_hole)
Y_prime = -z_hole * Y / (Z - z_hole)
Z_prime = np.ones_like(X) * z_hole

# 创建3D图形对象
fig = plt.figure()
ax = fig.gca(projection='3d')

# 绘制物体和小孔
ax.plot_surface(X, Y, Z, color='gray', alpha=0.5)
ax.scatter(x_hole, y_hole, z_hole, color='red', marker='o', s=100)

# 绘制光线
for i in range(len(x)):
    ax.plot([x[i], x_hole], [y[i], y_hole], [0, z_hole], color='blue', alpha=0.2)

# 绘制成像平面和图像
ax.plot_surface(X_image, Y_image, Z_image, color='gray', alpha=0.5)
ax.scatter(X_prime, Y_prime, Z_prime, color='green', marker='o', s=10)

# 设置坐标轴范围和标签
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(0, 10)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# 显示图形
plt.show()
