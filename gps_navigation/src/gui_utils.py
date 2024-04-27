#!/usr/bin/python3
import sys
import os
import csv
import pyqtgraph as pg

# add pxrf's plot script to lookup path
import rospkg
rospack = rospkg.RosPack()
work_dir_path = os.path.join(rospack.get_path('gps_navigation'), 'src')
sys.path.insert(0, work_dir_path)

def read_location(map_option=3):
    print("Please ensure that the robot is connected to the wifi.")
    data = []
    num = 0
    data_file = os.path.join(work_dir_path, 'locations.csv')
    with open(data_file, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter = ',')
        for row in reader:
            if row:
                num += 1
                data.append(row)
        print("Current Map: " + data[-1][0] + "\n")
        
    # while(True):
        # map_option = input(" \n Select the following options: \n 1.change map \n 2.new map \n 3.continue \n")
    
        if map_option == 1:
            print("Select from the following map: \n")
            for i in range(len(data)):
                print(str(i+1) + ". " + data[i][0])
            select2 = input()
            if not (int(select2) >= 1 and int(select2) <= num):
                print("invalid number")
            else:
                return data[int(select2) - 1]
        elif map_option == 2:
            filename = input("enter the map name: ")
            lat = input("enter the latitude: ")
            lon = input("enter the longitude: ")
            zoom = input("enter the zoom - recommend 20: ")
            width = input("enter the width - recommend 10: ")
            height = input("enter the height - recommend 10: ")
            new_location = [filename + "," + lat + "," + lon + "," + zoom + "," + width + "," + height]
            with open(data_file, 'a') as writefile:
                writer = csv.writer(writefile)
                writer.writerow([filename, lat, lon, zoom, width, height])
            print("new location added successfully")
            return [filename, lat, lon, zoom, width, height]
        elif map_option == 3:
            # print("Launching GUI")
            return data[-1]


class PlotWithClick(pg.PlotItem):
    def mouseClickEvent(self, ev):
        xClick = self.getViewBox().mapSceneToView(ev.scenePos()).x()
        yClick = self.getViewBox().mapSceneToView(ev.scenePos()).y()
        for handler in self.click_handlers:
            handler([xClick, yClick])
            
class MeasurementMarker(pg.GraphicsObject):
    def __init__(self, x, y, parent=None):
        super().__init__(parent)
        size = 50
        self._rect = QtCore.QRectF(x - size // 2, y - size // 2, size, size)
        self.picture = QtGui.QPicture()
        self._generate_picture()

    @property
    def rect(self):
        return self._rect

    def _generate_picture(self):
        painter = QtGui.QPainter(self.picture)
        painter.setPen(pg.mkPen("w"))
        painter.setBrush(pg.mkBrush("g"))
        painter.drawRect(self.rect)
        painter.end()

    def paint(self, painter, option, widget=None):
        painter.drawPicture(0, 0, self.picture)

    def boundingRect(self):
        return QtCore.QRectF(self.picture.boundingRect())


class PolyLineROINoHover(pg.PolyLineROI):
    def hoverEvent(self, ev):
        pass
