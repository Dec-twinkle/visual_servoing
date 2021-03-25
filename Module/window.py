# _*_ coding:utf-8 _*_
# @time: 2021/3/24 上午9:08
# @author: 张新新
# @email: 1262981714@qq.com
import open3d


class windowUtils():
    width = 640
    height = 480
    left = 10
    top = 10
    i = 0
    def getWindow(self,windowsName):
        windowParam = {"windowsName":windowsName,
                       "width":self.width,
                       "height":self.height,
                        "left":self.left,
                       "top":self.top}
        if self.i==3:
            self.i=0
            self.top=self.top+self.height
            self.left=10
        else:
            self.left= self.left+self.width
        return windowParam
