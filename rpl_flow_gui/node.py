import traceback
from pprint import pprint

import cv2
import numpy as np
from PIL import Image
from PIL.ImageQt import ImageQt
from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import (QBrush, QColor, QFont, QImage, QLinearGradient,
                         QPainterPath, QPalette, QPen, QPixmap)
from PyQt5.QtWidgets import (QComboBox, QFileDialog, QFrame, QGraphicsItem,
                             QGraphicsPathItem, QGraphicsProxyWidget,
                             QGridLayout, QHBoxLayout, QLabel, QLineEdit,
                             QPushButton, QSlider, QVBoxLayout, QWidget)

NODE_VERBOSE_ERRORS = False
NODE_MAX_PREVIEW_SIZE = 200
SOCKET_COLOR = (40, 120, 250)
SOCKET_IN = 1
SOCKET_OUT = 2

img = np.ones((51, 51, 3), dtype=np.uint8) * 32
cv2.circle(img, center=(25, 25), radius=17, color=SOCKET_COLOR, thickness=5, lineType=cv2.LINE_AA, shift=0)
cv2.circle(img, center=(25, 25), radius=20, color=(0, 0, 0), thickness=2, lineType=cv2.LINE_AA, shift=0)
img = cv2.resize(img, (18, 18))
linkNoneImg = QImage(ImageQt(Image.fromarray(img))).convertToFormat(QImage.Format_ARGB32).rgbSwapped()

img = np.ones((51, 51, 3), dtype=np.uint8) * 32
cv2.circle(img, center=(25, 25), radius=20, color=SOCKET_COLOR, thickness=-1, lineType=cv2.LINE_AA, shift=0)
cv2.circle(img, center=(25, 25), radius=20, color=(0, 0, 0), thickness=2, lineType=cv2.LINE_AA, shift=0)
cv2.line(img, pt1=(25, 25), pt2=(0, 25), color=SOCKET_COLOR, thickness=3, lineType=cv2.LINE_AA, shift=0)
img = cv2.resize(img, (18, 18))
linkLeftImg = QImage(ImageQt(Image.fromarray(img))).convertToFormat(QImage.Format_ARGB32).rgbSwapped()

img = np.ones((51, 51, 3), dtype=np.uint8) * 32
cv2.circle(img, center=(25, 25), radius=20, color=SOCKET_COLOR, thickness=-1, lineType=cv2.LINE_AA, shift=0)
cv2.circle(img, center=(25, 25), radius=20, color=(0, 0, 0), thickness=2, lineType=cv2.LINE_AA, shift=0)
cv2.line(img, pt1=(25, 25), pt2=(51, 25), color=SOCKET_COLOR, thickness=3, lineType=cv2.LINE_AA, shift=0)
img = cv2.resize(img, (18, 18))
linkRightImg = QImage(ImageQt(Image.fromarray(img))).convertToFormat(QImage.Format_ARGB32).rgbSwapped()

class FileField(QPushButton):
    def __init__(self, title, _):
        super(FileField, self).__init__('File...')

        self.title = title
        self._value = None

        self.clicked.connect(self.recalculate)

    def get_value(self):
        return self._value

    def recalculate(self):
        fileName, _filter = QFileDialog.getOpenFileName(self, 'Open File')
        self._value = fileName
        self.parent().recalculate()

    def get_save_state(self):
        return str(self._value)

    def set_save_state(self, state):
        self._value = str(state)

class SliderField(QSlider):
    def __init__(self, title, params):
        super(SliderField, self).__init__(Qt.Horizontal)

        self.mn = params['min']
        self.mx = params['max']

        self.setMinimum(0)
        self.setMaximum(100)
        self.setValue(params['default'])
        self.setFixedWidth(75)

        self.title = title

        self.valueChanged.connect(self.recalculate)

    def get_value(self):
        return np.interp(self.value(), [0, 100], [self.mn, self.mx])

    def recalculate(self):
        self.parent().recalculate()

    def get_save_state(self):
        return str(self.value())

    def set_save_state(self, state):
        self.setValue(int(state))

class StringField(QLineEdit):
    def __init__(self, title, _):
        super(StringField, self).__init__()

        self.setFixedWidth(75)

        self.title = title

        self.editingFinished.connect(self.recalculate)

    def get_value(self):
        return self.text()

    def recalculate(self):
        self.parent().recalculate()

    def get_save_state(self):
        return str(self.text())

    def set_save_state(self, state):
        self.setText(str(state))

class ChoiceField(QComboBox):
    def __init__(self, title, params):
        super(ChoiceField, self).__init__()

        self.values = []
        for choiceKey, choiceVal in params['choices'].items():
            self.addItem(choiceKey)
            self.values.append(choiceVal)

        self.setFixedWidth(75)

        self.title = title

        self.currentIndexChanged.connect(self.recalculate)

    def get_value(self):
        return self.values[self.currentIndex()]

    def recalculate(self):
        self.parent().recalculate()

    def get_save_state(self):
        return str(self.currentIndex())

    def set_save_state(self, state):
        self.setCurrentIndex(int(state))

class DebugField(QPushButton):
    def __init__(self, title, _):
        super(DebugField, self).__init__('Debug')

        self.title = title

        self.clicked.connect(self.recalculate)

    def get_value(self):
        return None

    def recalculate(self):
        self.parent().recalculate()
        pprint(self.parent().outputCache)

    def get_save_state(self):
        return str('')

    def set_save_state(self, state):
        pass

fieldWidgets = {
    'file': FileField,
    'slider': SliderField,
    'string': StringField,
    'choice': ChoiceField,
    'debug': DebugField,
}

class NodeContainer(QGraphicsItem):
    def __init__(self, path, title, func, fields, inputs, outputs, color, simple, *args, **kwargs):
        super(NodeContainer, self).__init__(*args, **kwargs)

        self.path = path
        # TODO: VERY TEMP
        self.title = title

        self.fields = []
        self.inputs = []
        self.outputs = []

        self.node = Node(self, title, func, fields, inputs, outputs, simple)
        self.grContent = QGraphicsProxyWidget(self)
        self.grContent.setWidget(self.node)

        # UI
        self.setFlag(QGraphicsItem.ItemIsSelectable)
        self.setFlag(QGraphicsItem.ItemIsMovable)

        self.corner_radius = 5
        self.reset_geometry()

        self._pen_default = QPen(QColor('#000000'))
        self._pen_selected = QPen(QColor('#00ff00'))

        gradient = QLinearGradient(0, 0, self.width, 0)
        gradient.setColorAt(0, QColor.fromRgbF(color[0]*0.8, color[1]*0.8, color[2]*0.8, 1))
        gradient.setColorAt(1, QColor.fromRgbF(color[0], color[1], color[2], 1))
        self._brush_title = QBrush(gradient)
        # self._brush_title = QBrush(QColor('#000000'))
        self._brush_background = QBrush(QColor('#202020'))

    def reset_geometry(self):
        self.width = self.node.frameGeometry().width()
        self.height = self.node.frameGeometry().height()
        # TODO: Change this 10 into a variable
        self.title_height = 10 + self.node.titleLabel.frameGeometry().height()
        self.update()

    # def set_color(self, color):
        # self._brush_title.setColor(QColor(color))

        # r = (int(color[1:], 16) & (255<<16)) >> 16
        # g = (int(color[1:], 16) & (255<<8)) >> 8
        # b = (int(color[1:], 16) & (255))

        # if (r+g+b)/3 > 128:
        #     self.title_item.setDefaultTextColor(Qt.black)
        # else:
        #     self.title_item.setDefaultTextColor(Qt.white)

    def boundingRect(self):
        return QRectF(0, 0, self.width, self.height).normalized()

    def paint(self, painter, QStyleOptionGraphicsItem, widget=None):
        # Title
        path_title = QPainterPath()
        path_title.setFillRule(Qt.WindingFill)
        path_title.addRoundedRect(0, 0, self.width, self.title_height, self.corner_radius, self.corner_radius)
        path_title.addRect(0, self.title_height - self.corner_radius, self.width, self.corner_radius)
        painter.setPen(Qt.NoPen)
        painter.setBrush(self._brush_title)
        painter.drawPath(path_title.simplified())

        # Content
        path_content = QPainterPath()
        path_content.setFillRule(Qt.WindingFill)
        path_content.addRoundedRect(0, self.title_height, self.width, self.height - self.title_height, self.corner_radius, self.corner_radius)
        path_content.addRect(0, self.title_height, self.width, self.corner_radius)
        painter.setPen(Qt.NoPen)
        painter.setBrush(self._brush_background)
        painter.drawPath(path_content.simplified())

        # Outline
        path_outline = QPainterPath()
        path_outline.addRoundedRect(-.5, -.5, self.width+1, self.height+1, self.corner_radius, self.corner_radius)
        painter.setPen(self._pen_default if not self.isSelected() else self._pen_selected)
        painter.setBrush(Qt.NoBrush)
        painter.drawPath(path_outline.simplified())

class Node(QWidget):
    def __init__(self, proxy, title, func, fields, inputs, outputs, simple, *args, **kwargs):
        super(Node, self).__init__(*args, **kwargs)

        self.proxy = proxy
        self.title = title
        self.func = func
        self.simple = simple
        self.outputCache = []

        self._init_ui(fields, inputs, outputs)

        self.recalculate()

    def _init_ui(self, fields, inputs, outputs):
        self.setStyleSheet('Node { background-color: transparent; }\
                            QLabel { color: white; }')

        # Create layouts
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.setSpacing(0)

        self.titleLayout = QHBoxLayout()
        self.titleLayout.setContentsMargins(5, 5, 5, 5)
        self.titleLayout.setSpacing(5)

        self.ioLayout = QGridLayout()
        self.ioLayout.setContentsMargins(0, 0, 0, 0)
        self.ioLayout.setSpacing(5)
        self.ioLayout.setColumnStretch(1, 1)
        self.ioLayout.setColumnStretch(3, 1)

        self.hudLayout = QVBoxLayout()
        self.hudLayout.setContentsMargins(5, 5, 5, 5)
        self.hudLayout.setSpacing(5)

        self.layout.addLayout(self.titleLayout)
        self.layout.addLayout(self.ioLayout)
        self.hudFrame = QFrame()
        self.hudFrame.setLayout(self.hudLayout)
        self.layout.addWidget(self.hudFrame)

        # Create title bar widgets
        self.titleLabel = QLabel(self.title)
        font = QFont()
        fontSize = self.titleLabel.font().pointSize()
        if not self.simple:
            fontSize += 5
        font.setPointSize(fontSize)
        self.titleLabel.setFont(font)
        self.titleLayout.addWidget(self.titleLabel)

        # Create input/output widgets
        h = 2 + max(len(fields)+len(inputs), len(outputs))
        self.ioLayout.addWidget(QHLine(), 0, 0, 1, 5)
        self.ioLayout.addWidget(QVLine(), 0, 2, h+1, 1)
        self.ioLayout.addWidget(QHLine(), h, 0, 1, 5)

        if len(outputs) > 1:
            self.outputChoice = QComboBox()

        for i, field in enumerate(fields):
            label = QLabel(field['title'])
            label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            self.ioLayout.addWidget(label, 1+i, 1)

            field = fieldWidgets[field['widget']](field['title'], field)
            # field.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            self.ioLayout.addWidget(field, 1+i, 0)
            self.proxy.fields.append(field)

        for i, inp in enumerate(inputs):
            label = QLabel(inp['title'])
            label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            self.ioLayout.addWidget(label, 1+i+len(fields), 1)

            socket = Socket(SOCKET_IN, inp['title'])
            socket.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            self.ioLayout.addWidget(socket, 1+i+len(fields), 0)
            self.proxy.inputs.append(socket)

        for i, outp in enumerate(outputs):
            label = QLabel(outp['title'])
            label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.ioLayout.addWidget(label, 1+i, 3)

            socket = Socket(SOCKET_OUT, outp['title'])
            socket.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.ioLayout.addWidget(socket, 1+i, 4)
            self.proxy.outputs.append(socket)

            if len(outputs) > 1:
                self.outputChoice.addItem(outp['title'])

        # Create output HUD widgets
        if len(outputs) > 1:
            self.outputChoice.currentIndexChanged.connect(self.redisplay)
            self.hudLayout.addWidget(self.outputChoice)
        self.outputLabel = QLabel('-')
        self.outputLabel.setAlignment(Qt.AlignCenter)
        self.hudLayout.addWidget(self.outputLabel)

        if self.simple:
            self.hudFrame.hide()

    def get_value(self, socket):
        idx = self.proxy.outputs.index(socket)
        return self.outputCache[idx]

    def recalculate(self):
        try:
            # self.status = 'running'

            argumentDict = {}
            for socket in self.proxy.inputs:
                if socket.edges:
                    source = socket.edges[0].startSocket
                    argumentDict[socket.title] = source.parent().get_value(source)
                else:
                    argumentDict[socket.title] = None
            for field in self.proxy.fields:
                # TODO: When fields are a thing, get_value() needs to be written
                argumentDict[field.title] = field.get_value()

            self.outputCache = self.func(**argumentDict)
            if len(self.proxy.outputs) == 1:
                self.outputCache = [self.outputCache]
        except Exception as e:
            # self.status = 'error'

            if NODE_VERBOSE_ERRORS:
                print('====+ Traceback:', self.title)
                traceback.print_exc()
                print('====- Traceback:', self.title)

            # This line is not a mistake!
            # Intended to cause errors in downstream node recalculations
            self.outputCache = []
        else:
            pass # self.status = 'normal'
        finally:
            self.redisplay()

            for socket in self.proxy.outputs:
                for edge in socket.edges:
                    edge.endSocket.parent().recalculate()

    def redisplay(self):
        if not self.simple:
            if not self.outputCache:
                self.outputLabel.setText('-')
            else:
                if len(self.outputCache) == 1:
                    val = self.outputCache[0]
                else:
                    val = self.outputCache[self.outputChoice.currentIndex()]

                if isinstance(val, np.ndarray) and val.ndim in (2,3):
                    img = np.require(val, np.uint8, 'C')
                    mx = max(img.shape[:2])
                    img = cv2.resize(img, (NODE_MAX_PREVIEW_SIZE*img.shape[1]//mx, NODE_MAX_PREVIEW_SIZE*img.shape[0]//mx))
                    # img = QImage(img, img.shape[1], img.shape[0], QImage.Format_RGB888).rgbSwapped()
                    img = QImage(ImageQt(Image.fromarray(img))).convertToFormat(QImage.Format_ARGB32).rgbSwapped()
                    self.outputLabel.setPixmap(QPixmap(img))
                else:
                    text = str(val)
                    if len(text) > 10:
                        text = text[:10] + '...'
                    self.outputLabel.setText(text)

        self.adjustSize()
        self.resize(self.minimumSizeHint())
        QTimer.singleShot(10, self.proxy.reset_geometry)

class Socket(QLabel):
    def __init__(self, iotype, title):
        super(Socket, self).__init__()

        self.iotype = iotype
        self.title = title
        self.edges = []
        self.redisplay()

    def get_center(self):
        position = [
            self.pos().x() + self.parent().proxy.scenePos().x() + (self.width()//2),
            self.pos().y() + self.parent().proxy.scenePos().y() + (self.height()//2)
        ]

        return position

    def redisplay(self):
        if not self.edges:
            # .setDevicePixelRatio(2)
            self.setPixmap(QPixmap(linkNoneImg))
        else:
            if self.iotype == SOCKET_IN:
                self.setPixmap(QPixmap(linkLeftImg))
            else:
                self.setPixmap(QPixmap(linkRightImg))

class Edge(QGraphicsPathItem):
    def __init__(self, startSocket, endSocket, *args, **kwargs):
        super(Edge, self).__init__(*args, **kwargs)

        self.startSocket = startSocket
        self.endSocket = endSocket

        # UI
        self._color = QColor('#fa7828') # QColor('#909090')
        self._color_selected = QColor('#00ff00')
        self._pen = QPen(self._color)
        self._pen_selected = QPen(self._color_selected)
        self._pen.setWidthF(2)
        self._pen_selected.setWidthF(2)
        self.lineType = 'bezier'

        self.setFlag(QGraphicsItem.ItemIsSelectable)

        self.setZValue(-1)

        self.update()

    def paint(self, painter, QStyleOptionGraphicsItem, widget=None):
        self.calculate_path()

        painter.setPen(self._pen if not self.isSelected() else self._pen_selected)
        painter.setBrush(Qt.NoBrush)
        painter.drawPath(self.path())

    def calculate_path(self):
        start = self.startSocket.get_center()
        end = self.endSocket.get_center()

        bezier_path(self, start, end)

class FakeEdge(QGraphicsPathItem):
    def __init__(self, flip, *args, **kwargs):
        super(FakeEdge, self).__init__(*args, **kwargs)

        self.flip = flip
        self.posSource = [0, 0]
        self.posDestination = [0, 0]

        # UI
        self._color = QColor('#909090')
        self._color_selected = QColor('#00ff00')
        self._color_dragging = QColor('#ffffff')
        self._pen = QPen(self._color)
        self._pen_selected = QPen(self._color_selected)
        self._pen_dragging = QPen(self._color_dragging)
        self._pen.setWidthF(2)
        self._pen_selected.setWidthF(2)
        self._pen_dragging.setWidthF(2)
        self.lineType = 'bezier'

        self.setFlag(QGraphicsItem.ItemIsSelectable)

        self.setZValue(-1)

        self.update()

    def paint(self, painter, QStyleOptionGraphicsItem, widget=None):
        self.calculate_path()

        painter.setPen(self._pen_dragging)
        painter.setBrush(Qt.NoBrush)
        painter.drawPath(self.path())

    def calculate_path(self):
        if self.flip == SOCKET_IN:
            start = self.posDestination
            end = self.posSource
        else:
            start = self.posSource
            end = self.posDestination

        bezier_path(self, start, end)

class QHLine(QFrame):
    def __init__(self):
        super(QHLine, self).__init__()
        self.setFrameShape(QFrame.HLine)
        self.setFrameShadow(QFrame.Sunken)
        p = QPalette(QColor(), QColor(), QColor(Qt.black), QColor(Qt.black), QColor(), QColor(), QColor(), QColor(), QColor())
        self.setPalette(p)

class QVLine(QFrame):
    def __init__(self):
        super(QVLine, self).__init__()
        self.setFrameShape(QFrame.VLine)
        self.setFrameShadow(QFrame.Sunken)
        p = QPalette(QColor(), QColor(), QColor(Qt.black), QColor(Qt.black), QColor(), QColor(), QColor(), QColor(), QColor())
        self.setPalette(p)

def bezier_path(pathItem, start, end):
    offset = 15

    # TODO: string is bad
    if pathItem.lineType == 'bezier':
        if start[0] < end[0]:
            dist = (end[0] - start[0]) / 2
        else:
            dist = -(end[0] - start[0]) / 2

        cpStart = [dist, 0]
        cpEnd = [-dist, 0]

        path = QPainterPath(QPointF(start[0], start[1]))
        path.lineTo(start[0]+offset, start[1])
        path.cubicTo(start[0]+offset+cpStart[0], start[1]+cpStart[1], end[0]-offset+cpEnd[0], end[1]+cpEnd[1], end[0]-offset, end[1])
        path.lineTo(end[0], end[1])
        pathItem.setPath(path)
    else:
        path = QPainterPath(QPointF(start[0], start[1]))
        path.lineTo(start[0]+offset, start[1])
        path.lineTo(end[0]-offset, end[1])
        path.lineTo(end[0], end[1])
        pathItem.setPath(path)
