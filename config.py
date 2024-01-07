from PyQt5.QtCore import QPoint

WIDTH = 1300
HEIGHT = 700

#A-----B
#|     |
#C-----D
pointA = [50,50]
pointB = [WIDTH-100,50]
pointC = [50,HEIGHT-100]
pointD = [WIDTH-100,HEIGHT-100]

COLOR_NAMES = ['red','green','blue']

obstacle1_n = [[210,250],[260,180],[320,240]]
obstacle2_n = [[500,400],[640,410],[590,480]]
obstacle3_n = [[800,300],[860,200],[960,290]]

obstacle1 = ([
    QPoint(obstacle1_n[0][0],obstacle1_n[0][1]),
    QPoint(obstacle1_n[1][0],obstacle1_n[1][1]),
    QPoint(obstacle1_n[2][0],obstacle1_n[2][1])
])

obstacle2 = ([
    QPoint(obstacle2_n[0][0],obstacle2_n[0][1]),
    QPoint(obstacle2_n[1][0],obstacle2_n[1][1]),
    QPoint(obstacle2_n[2][0],obstacle2_n[2][1])
])

obstacle3 = ([
    QPoint(obstacle3_n[0][0],obstacle3_n[0][1]),
    QPoint(obstacle3_n[1][0],obstacle3_n[1][1]),
    QPoint(obstacle3_n[2][0],obstacle3_n[2][1])
])

BLOCK_POSITIONS = [
    (0.1,0.1), #0 - Start
    (1.1,0.03), #1 - Bucket Red 
    (1.1,0.265), #2 - Bucket Green
    (1.1,0.5), #3 - Bucket Blue
    (0.2,0.07), #1
    (0.20,0.3), #2
    (0.28,0.3), #3
    (0.35,0.15), #4
    (0.37,0.06), #5
    (0.5,0.28), #6
    (0.68,0.06), #7
    (0.83,0.06), #8
    (0.93,0.07), #9
    (1.00,0.16), #10
    (1.00,0.4), #11
    (0.91,0.50), #12
    (0.66,0.38), #13
    (0.67,0.48), #14
    (0.5,0.46), #15
    (0.35,0.45), #16
    (0.06,0.45), #17
    (0.01,0.27), #18
    (0.02,0.15), #19
    (0.81,0.24), #20
]