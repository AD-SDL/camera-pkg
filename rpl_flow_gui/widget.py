from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QPainter
from PyQt5.QtWidgets import (QFileDialog, QGraphicsProxyWidget, QGraphicsScene,
                             QGraphicsView, QMenu, QVBoxLayout, QWidget)

from node import SOCKET_IN, SOCKET_OUT, Edge, FakeEdge, NodeContainer, Socket


class NodeEditorWidget(QWidget):
    def __init__(self, *args, **kwargs):
        super(NodeEditorWidget, self).__init__(*args, **kwargs)

        self.nodes = []
        self.edges = []
        self.registeredNodes = {}
        self.clickPos = None
        self.tree = {}
        self.menu = QMenu()
        self.fname = None

        action = self.menu.addAction('Save Graph')
        action.triggered.connect(self.save)
        action = self.menu.addAction('Load Graph')
        action.triggered.connect(self.load)

        # UI
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)

        self.scene = QGraphicsScene(self)
        self.sceneWidth, self.sceneHeight = 5000, 5000
        self.scene.setSceneRect(-self.sceneWidth//2, -self.sceneHeight//2, self.sceneWidth, self.sceneHeight)
        self.scene.setBackgroundBrush(QColor('#393939'))

        self.view = NodeEditorGraphicsView(self.scene, self)
        self.layout.addWidget(self.view)

    def save(self):
        if self.fname is None:
            fname, _filter = QFileDialog.getSaveFileName(self, 'Save File', filter='*.graph')
            if fname is '':
                return

            self.fname = fname

        with open(self.fname, 'w') as f:
            for node in self.nodes:
                print('N', id(node), node.pos().x(), node.pos().y(), node.path, file=f)
                for fieldNum, field in enumerate(node.fields):
                    print('F', id(node), fieldNum, field.get_save_state(), file=f)

            for node1 in self.nodes:
                for sockNum1, socket in enumerate(node1.outputs):
                    for edge in socket.edges:
                        node2 = edge.endSocket.parent().proxy
                        sockNum2 = node2.inputs.index(edge.endSocket)
                        print('E', id(node1), sockNum1, id(node2), sockNum2, file=f)

    def load(self):
        fname, _filter = QFileDialog.getOpenFileName(self, 'Open File')
        if fname is '':
            return

        self.fname = fname

        id2node = {}
        with open(self.fname) as f:
            for line in f:
                if line[0] == 'N':
                    _, nodeID, posX, posY, *path = line.split()

                    path = ' '.join(path)
                    node = self.registeredNodes[path][1]()
                    node.setPos(float(posX), float(posY))
                    id2node[int(nodeID)] = node
                elif line[0] == 'F':
                    _, nodeID, fieldNum, state = line.split()

                    node = id2node[int(nodeID)]
                    node.fields[int(fieldNum)].set_save_state(state)
                elif line[0] == 'E':
                    _, nodeID1, socketNum1, nodeID2, socketNum2 = line.split()

                    node1 = id2node[int(nodeID1)]
                    node2 = id2node[int(nodeID2)]
                    edge = Edge(node1.outputs[int(socketNum1)], node2.inputs[int(socketNum2)])
                    self.add_edge(edge)

        for node in id2node.values():
            node.node.recalculate()

    def register(self, path, plugin):
        longpath = path+'/'+plugin['title']
        if longpath in self.registeredNodes:
            print('Already registered', longpath)
            return

        assert longpath == ' '.join(longpath.split())

        plugin.setdefault('fields', [])
        plugin.setdefault('inputs', [])
        plugin.setdefault('outputs', [])
        plugin.setdefault('simple', False)
        plugin.setdefault('color', (0.0, 0.0, 0.0))

        def create_node():
            n = NodeContainer(longpath, **plugin)
            n.setPos(self.clickPos)
            self.add_node(n)
            return n

        self.registeredNodes[longpath] = (plugin, create_node)

        currMenu, currTree = self.menu, self.tree
        for label in path.split('/'):
            if label not in currTree:
                tempMenu, tempTree = currMenu.addMenu(label), {}
                currTree[label] = (tempMenu, tempTree)
                currMenu, currTree = tempMenu, tempTree
            else:
                currMenu, currTree = currTree[label]

        action = currMenu.addAction(plugin['title'])
        action.triggered.connect(self.registeredNodes[longpath][1])

    def add_node(self, node):
        self.scene.addItem(node)
        self.nodes.append(node)

    def remove_node(self, node):
        for socket in (node.inputs + node.outputs):
            for edge in socket.edges[:]:
                self.remove_edge(edge)

        # TODO: Also remove widget with official means
        self.scene.removeItem(node)
        self.nodes.remove(node)

    def add_edge(self, edge):
        # TODO: Remove existing edges on sockets when necessary
        # TODO: Check if graph has loops / other issues

        for e in edge.endSocket.edges[:]:
            self.remove_edge(e)

        edge.startSocket.edges.append(edge)
        edge.endSocket.edges.append(edge)

        self.scene.addItem(edge)
        self.edges.append(edge)

        edge.endSocket.parent().recalculate()

        edge.startSocket.redisplay()
        edge.endSocket.redisplay()

        # if self.iotype == '?put':
        #     for e in self.edges[:]:
        #         if edge.startSocket == e.startSocket:
        #             self.node.nodeHandler.remove_edge(e)
        # elif self.iotype == '?put':
        #     for e in self.edges[:]:
        #         if edge.endSocket == e.endSocket:
        #             self.node.nodeHandler.remove_edge(e)

        # if not self.multiEdge:
        #     for edgeToDelete in self.edges:
        #         self.node.nodeHandler.remove_edge(edgeToDelete)

        # self.edges.append(edge)

    def remove_edge(self, edge):
        edge.startSocket.edges.remove(edge)
        edge.endSocket.edges.remove(edge)

        # TODO: Also remove widget with official means
        self.scene.removeItem(edge)
        self.edges.remove(edge)

        edge.endSocket.parent().recalculate()

        # TODO: Is this necessary?
        # edge.startSocket = None
        # edge.endSocket = None

        edge.startSocket.redisplay()
        edge.endSocket.redisplay()

    def add_fake_edge(self, edge):
        self.scene.addItem(edge)
        self.edges.append(edge)

    def remove_fake_edge(self, edge):
        # TODO: Also remove widget with official means
        self.scene.removeItem(edge)
        self.edges.remove(edge)

    def contextMenuEvent(self, event):
        self.clickPos = self.view.mapToScene(event.pos())
        self.menu.exec(event.globalPos())
        self.clickPos = None

class NodeEditorGraphicsView(QGraphicsView):
    def __init__(self, scene, nodeHandler, *args, **kwargs):
        super(NodeEditorGraphicsView, self).__init__(nodeHandler, *args, **kwargs)

        # TODO: What is this?
        # self.setContextMenuPolicy(Qt.ActionsContextMenu)

        self.nodeHandler = nodeHandler

        self.lastMousePos = [0, 0]
        self.makingEdge = False
        self.fakeEdge = None
        self.fakeEdgeSocket = None

        self.setScene(scene)

        # UI
        self.setRenderHints(QPainter.Antialiasing | \
                            QPainter.HighQualityAntialiasing | \
                            QPainter.TextAntialiasing | \
                            QPainter.SmoothPixmapTransform)

        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)

        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setDragMode(QGraphicsView.RubberBandDrag)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.leftMousePress(event)
        else:
            super(NodeEditorGraphicsView, self).mousePressEvent(event)

    def mouseMoveEvent(self, event):
        self.lastMousePos = self.mapToScene(event.pos())
        self.lastMousePos = [self.lastMousePos.x(), self.lastMousePos.y()]

        if self.makingEdge:
            self.fakeEdge.posDestination = self.lastMousePos
            self.fakeEdge.update()

        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.leftMouseRelease(event)
        else:
            super(NodeEditorGraphicsView, self).mouseReleaseEvent(event)

    def leftMousePress(self, event):
        item = self.itemAt(event.pos())

        if isinstance(item, QGraphicsProxyWidget):
            wid = item.widget()
            socket = wid.childAt(wid.mapFromGlobal(event.globalPos()))

            if isinstance(socket, Socket):
                self.makingEdge = True

                self.fakeEdge = FakeEdge(socket.iotype)
                self.fakeEdge.posSource = socket.get_center()
                self.fakeEdge.posDestination = self.lastMousePos
                self.fakeEdgeSocket = socket

                self.nodeHandler.add_fake_edge(self.fakeEdge)

                return

        super(NodeEditorGraphicsView, self).mousePressEvent(event)

    def leftMouseRelease(self, event):
        item = self.itemAt(event.pos())

        if self.makingEdge:
            self.makingEdge = False

            if isinstance(item, QGraphicsProxyWidget):
                wid = item.widget()
                socket = wid.childAt(wid.mapFromGlobal(event.globalPos()))

                if isinstance(socket, Socket) and socket is not self.fakeEdgeSocket:
                    # TODO: Does not confirm 1 input and 1 output
                    if socket.iotype == SOCKET_IN:
                        newEdge = Edge(self.fakeEdgeSocket, socket)
                    elif socket.iotype == SOCKET_OUT:
                        newEdge = Edge(socket, self.fakeEdgeSocket)

                    self.nodeHandler.add_edge(newEdge)

            self.nodeHandler.remove_fake_edge(self.fakeEdge)
            self.fakeEdge = None

            return

        super(NodeEditorGraphicsView, self).mouseReleaseEvent(event)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Backspace:
            selected = self.scene().selectedItems()
            if selected:
                for item in selected:
                    if isinstance(item, NodeContainer):
                        self.nodeHandler.remove_node(item)
            else:
                super(NodeEditorGraphicsView, self).keyPressEvent(event)
        # elif event.key() == Qt.Key_S and event.modifiers() & Qt.ControlModifier:
        #     self.nodeHandler.save_to_file('graph.json')
        # elif event.key() == Qt.Key_L and event.modifiers() & Qt.ControlModifier:
        #     self.nodeHandler.load_from_file('graph.json')
        else:
            super(NodeEditorGraphicsView, self).keyPressEvent(event)
